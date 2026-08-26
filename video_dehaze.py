#!/usr/bin/env python3
"""
video_dehaze.py — dehaze.py의 DarkChannelDehazer를 영상(파일/웹캠)에 적용하는 CLI.

이미지용 process()는 프레임 하나만 받으므로, 영상은 프레임 단위로 읽어
process()를 반복 호출하고 다시 이어붙이면 됩니다. 이 스크립트가 그 루프와
부가 기능(오디오 유지, 진행률/FPS 표시, 원본과 좌우 비교, 저조도 모드)을
감싸줍니다.

기본 사용법:
    python3 video_dehaze.py input.mp4 output.mp4

주요 옵션:
    --scale 0.25          다크채널 추정 축소 배율 (dehaze.py의 실시간성 핵심 파라미터)
    --a-smoothing 0.85     대기광 A를 프레임 간 평활 (영상에서는 켜는 걸 강력 권장:
                           안 켜면 같은 물체가 프레임마다 다른 밝기로 복원되어
                           깜빡임(flicker)이 보입니다 — dehaze.py 주석 참고)
    --lowlight             DCP 대신 반전-디헤이즈-반전으로 저조도 보정
    --clahe                디헤이즈 후 LAB L채널 CLAHE 추가 적용
    --gamma 1.0            디헤이즈 후 감마 보정 (<1 밝게, >1 어둡게)
    --side-by-side         원본|결과 좌우 비교 영상으로 출력
    --no-audio              원본 오디오 트랙을 결과 영상에 다시 입히지 않음
    --max-frames N          앞 N프레임만 처리 (미리보기/튜닝용, 전체 처리 전 확인)
    --benchmark             처리 완료 후 평균 처리 FPS 출력 (실시간성 확인용)

오디오는 cv2.VideoWriter가 다루지 못하므로, ffmpeg가 있으면 처리가 끝난 뒤
원본의 오디오 트랙을 결과 영상에 다시 입힙니다(영상은 새로 인코딩된 것,
오디오만 원본에서 복사). ffmpeg가 없거나 원본에 오디오가 없으면 자동으로
건너뛰고 무음 결과만 남깁니다.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from dehaze import ClaheEnhancer, DarkChannelDehazer, apply_gamma


def format_eta(seconds: float) -> str:
    if seconds != seconds or seconds in (float("inf"), float("-inf")):
        return "?"
    m, s = divmod(int(seconds), 60)
    h, m = divmod(m, 60)
    return f"{h:02d}:{m:02d}:{s:02d}"


def print_progress(i: int, total: int, t_start: float) -> None:
    elapsed = time.time() - t_start
    fps = i / elapsed if elapsed > 0 else 0.0
    if total > 0:
        pct = i / total * 100
        eta = (total - i) / fps if fps > 0 else float("nan")
        msg = f"\r[{i}/{total}] {pct:5.1f}%  {fps:5.1f} fps  ETA {format_eta(eta)}"
    else:
        msg = f"\r[{i}] {fps:5.1f} fps"
    sys.stdout.write(msg)
    sys.stdout.flush()


def build_dehazer(args: argparse.Namespace) -> DarkChannelDehazer:
    return DarkChannelDehazer(
        omega=args.omega,
        t0=args.t0,
        patch=args.patch,
        scale=args.scale,
        use_guided=not args.no_guided,
        guided_radius=args.guided_radius,
        guided_eps=args.guided_eps,
        a_top_ratio=args.a_top_ratio,
        a_max=args.a_max,
        sky_ratio=args.sky_ratio,
        a_smoothing=args.a_smoothing,
    )


def mux_audio(silent_video: Path, source_with_audio: Path, final_out: Path) -> bool:
    """ffmpeg로 silent_video(새로 인코딩된 무음 영상)에 source_with_audio의
    오디오 트랙을 입혀 final_out으로 저장. 성공하면 True, 실패/오디오없음이면 False.
    """
    if shutil.which("ffmpeg") is None:
        return False
    cmd = [
        "ffmpeg", "-y",
        "-i", str(silent_video),
        "-i", str(source_with_audio),
        "-map", "0:v:0",
        "-map", "1:a:0?",
        "-c:v", "copy",
        "-c:a", "aac",
        "-shortest",
        str(final_out),
    ]
    result = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    if result.returncode != 0:
        return False
    # 오디오 스트림이 없어도 -map ...? 덕분에 ffmpeg는 성공 종료할 수 있으므로,
    # 실제로 오디오가 붙었는지 ffprobe로 한 번 더 확인.
    probe = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "a", "-show_entries",
         "stream=index", "-of", "csv=p=0", str(final_out)],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
    )
    return bool(probe.stdout.strip())


def main() -> int:
    p = argparse.ArgumentParser(description="Dark Channel Prior 기반 영상 디헤이징")
    p.add_argument("input", help="입력 영상 경로 (또는 웹캠 index, 예: 0)")
    p.add_argument("output", help="출력 영상 경로 (.mp4 권장)")

    dcp = p.add_argument_group("DCP 파라미터 (dehaze.py DarkChannelDehazer와 동일)")
    dcp.add_argument("--omega", type=float, default=0.95)
    dcp.add_argument("--t0", type=float, default=0.1)
    dcp.add_argument("--patch", type=int, default=15)
    dcp.add_argument("--scale", type=float, default=0.25, help="추정 축소 배율 (실시간성 핵심)")
    dcp.add_argument("--no-guided", action="store_true", help="guided filter 끄기")
    dcp.add_argument("--guided-radius", type=int, default=8)
    dcp.add_argument("--guided-eps", type=float, default=1e-3)
    dcp.add_argument("--a-top-ratio", type=float, default=0.001)
    dcp.add_argument("--a-max", type=float, default=0.92)
    dcp.add_argument("--sky-ratio", type=float, default=1.0)
    dcp.add_argument(
        "--a-smoothing", type=float, default=0.85,
        help="대기광 A의 프레임 간 EMA 계수 (0=끔, 영상은 0.7~0.9 권장, 정지영상 비교면 0)",
    )

    mode = p.add_argument_group("모드/후처리")
    mode.add_argument("--lowlight", action="store_true", help="저조도 보정 모드 (반전-디헤이즈-반전)")
    mode.add_argument("--lowlight-omega", type=float, default=0.8)
    mode.add_argument("--lowlight-t0", type=float, default=0.25)
    mode.add_argument("--clahe", action="store_true", help="디헤이즈 후 CLAHE 추가 적용")
    mode.add_argument("--clahe-clip", type=float, default=2.0)
    mode.add_argument("--clahe-tile", type=int, default=8)
    mode.add_argument("--gamma", type=float, default=1.0, help="디헤이즈 후 감마 보정 (기본 1.0=없음)")
    mode.add_argument("--side-by-side", action="store_true", help="원본|결과 좌우 비교 영상 출력")

    io_ = p.add_argument_group("입출력")
    io_.add_argument("--fourcc", default="mp4v", help="VideoWriter FourCC (기본 mp4v)")
    io_.add_argument("--no-audio", action="store_true", help="원본 오디오를 결과에 입히지 않음")
    io_.add_argument("--max-frames", type=int, default=0, help="앞 N프레임만 처리 (0=전체)")
    io_.add_argument("--benchmark", action="store_true", help="완료 후 평균 처리 FPS 출력")

    args = p.parse_args()

    # 웹캠 index 지원 (숫자 문자열이면 int로 변환)
    src = int(args.input) if args.input.isdigit() else args.input
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f"입력을 열 수 없습니다: {args.input}", file=sys.stderr)
        return 1

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if args.max_frames > 0:
        total_frames = min(total_frames, args.max_frames) if total_frames > 0 else args.max_frames

    out_w = width * 2 if args.side_by_side else width
    out_path = Path(args.output)

    input_path = Path(args.input) if not isinstance(src, int) else None
    want_audio = (not args.no_audio) and input_path is not None
    # 오디오를 입히려면 일단 무음 영상을 임시 파일에 쓰고, 끝난 뒤 ffmpeg로 합칩니다.
    write_target = out_path.with_suffix(".silent.mp4") if want_audio else out_path

    fourcc = cv2.VideoWriter_fourcc(*args.fourcc)
    writer = cv2.VideoWriter(str(write_target), fourcc, fps, (out_w, height))
    if not writer.isOpened():
        print(f"출력 writer를 열 수 없습니다 (fourcc={args.fourcc}): {write_target}", file=sys.stderr)
        cap.release()
        return 1

    dehazer = build_dehazer(args)
    clahe = ClaheEnhancer(clip_limit=args.clahe_clip, tile_grid=(args.clahe_tile, args.clahe_tile)) if args.clahe else None

    print(f"입력: {args.input}  ({width}x{height} @ {fps:.2f}fps, 총 {total_frames or '?'} 프레임)")
    print(f"출력: {out_path}  (side-by-side={args.side_by_side}, lowlight={args.lowlight})")

    i = 0
    t_start = time.time()
    try:
        while True:
            if args.max_frames > 0 and i >= args.max_frames:
                break
            ok, frame = cap.read()
            if not ok:
                break

            if args.lowlight:
                result = dehazer.process_lowlight(frame, omega=args.lowlight_omega, t0=args.lowlight_t0)
            else:
                result = dehazer.process(frame)

            if clahe is not None:
                result = clahe.process(result)
            if abs(args.gamma - 1.0) > 1e-3:
                result = apply_gamma(result, args.gamma)

            if args.side_by_side:
                out_frame = np.hstack([frame, result])
            else:
                out_frame = result

            writer.write(out_frame)
            i += 1
            if i % 5 == 0 or i == total_frames:
                print_progress(i, total_frames, t_start)
    finally:
        cap.release()
        writer.release()

    elapsed = time.time() - t_start
    avg_fps = i / elapsed if elapsed > 0 else 0.0
    print(f"\n프레임 {i}개 처리 완료 ({elapsed:.1f}s, 평균 {avg_fps:.1f} fps)")
    if args.benchmark:
        print(f"[benchmark] scale={args.scale} patch={args.patch} guided={not args.no_guided} -> {avg_fps:.2f} fps")

    if want_audio:
        print("오디오 트랙 병합 시도 중 (ffmpeg)...")
        ok = mux_audio(write_target, input_path, out_path)
        if ok:
            write_target.unlink(missing_ok=True)
            print(f"오디오 포함 결과 저장: {out_path}")
        else:
            # 오디오가 없거나 ffmpeg 실패 -> 무음 결과를 최종 경로로 사용
            write_target.replace(out_path)
            print(f"오디오를 붙이지 못해 무음 영상으로 저장했습니다: {out_path}")
            if shutil.which("ffmpeg") is None:
                print("  (ffmpeg가 설치되어 있지 않습니다)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())