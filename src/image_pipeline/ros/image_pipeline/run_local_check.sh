#!/usr/bin/env bash
# 로컬 기능 검증 한 방에 돌리기 — ROS 없이.
#
#   bash run_local_check.sh
#
# 순서: 의존성 확인 -> 합성 데이터 -> pytest -> 비교 이미지 -> 벤치 -> 태스크② 장면
# 전부 통과하면 알고리즘은 검증된 것이고, 이후 문제는 ROS 배선으로 범위가 좁혀집니다.

set -e
cd "$(dirname "$0")"

OUT="${1:-local_check}"

echo "=============================================="
echo " image_pipeline 로컬 기능 검증"
echo "=============================================="

echo
echo "[0/5] 의존성 확인"
python3 - <<'PY'
import sys
missing = []
for m in ("cv2", "numpy", "pytest"):
    try:
        __import__(m)
    except ImportError:
        missing.append({"cv2": "opencv-python"}.get(m, m))
if missing:
    print("  설치 필요:  pip install " + " ".join(missing))
    sys.exit(1)
import cv2, numpy
print(f"  OpenCV {cv2.__version__} / NumPy {numpy.__version__}  OK")
PY

echo
echo "[1/5] 합성 테스트 데이터 생성 (정답을 아는 입력)"
python3 tools/make_synthetic.py -o "$OUT/testdata"

echo
echo "[2/5] pytest — 알고리즘 정확성 검증"
python3 -m pytest tests/ -q

echo
echo "[3/5] 4조건 비교 이미지 + 정량 지표"
python3 tools/tune_offline.py "$OUT/testdata/hazy.png" "$OUT/testdata/dark_hazy.png" \
        -o "$OUT" --dump-transmission

echo
echo "[4/5] 해상도별 성능 (process_width 결정 근거)"
python3 tools/tune_offline.py "$OUT/testdata/hazy.png" --bench

echo
echo "[5/5] 태스크② 더미 장면 (박스 -> 거리 -> base_link)"
python3 tools/preview_depth_scene.py --box-y 60 -o "$OUT"
python3 tools/preview_depth_scene.py --floor 0.35 --flame-hole --region below -o "$OUT/floor"

echo
echo "=============================================="
echo " 완료. 결과물: $OUT/"
echo "   testdata/          합성 입력 + 정답(groundtruth.npz)"
echo "   *_compare.png      4조건 비교"
echo "   *_transmission.png 투과율 맵
   depth_scene.png    태스크② 더미 장면 (초록=project_box 정답 / 빨강=scale_box 함정)"
echo
echo " 다음: 실제 연기 사진으로도 돌려보세요"
echo "   python3 tools/tune_offline.py 내사진.jpg -o $OUT"
echo "   python3 tools/live_preview.py 내사진.jpg      # 트랙바 튜닝"
echo "=============================================="
