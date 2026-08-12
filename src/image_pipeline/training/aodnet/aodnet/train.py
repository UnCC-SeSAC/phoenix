#!/usr/bin/env python3
"""
AOD-Net 학습.

    python -m aodnet.train --epochs 30 --out runs/base

기본값으로 그냥 돌리면 절차적 장면으로 학습합니다(데이터 준비 0단계).
실제 로봇 프레임이 생기면:

    python -m aodnet.train --clear-dir data/clear --depth-dir data/depth --out runs/real

학습 안정성에 관해 (여기서 사람들이 제일 많이 넘어집니다)
-----------------------------------------------------------
출력이 J = K·I - K + b 라 **K가 곱해지는 구조**입니다. 그래서
  - 초기 K가 크면 곧바로 발산합니다 -> 가중치 std=0.02 초기화 (model.py)
  - 손실 스파이크 한 번에 K가 튀면 회복이 안 됩니다 -> grad clip 0.1 (논문값)
  - lr을 CNN 감각으로 1e-2쯤 주면 거의 확실히 NaN입니다
이 세 개 중 하나라도 빼면 loss가 nan으로 가고, 그때 보이는 증상은
"결과 이미지가 전부 검정"입니다. K가 0으로 죽어 ReLU에 막힌 상태입니다.
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from aodnet.data import PairedDataset, SyntheticSmokeDataset
from aodnet.device import device_name, empty_cache, pick_device, synchronize
from aodnet.metrics import psnr, ssim
from aodnet.model import AODNet, count_parameters
from aodnet.synth import SmokeConfig


def build_args():
    p = argparse.ArgumentParser(description="AOD-Net 화재연기 제거 학습")
    # 데이터
    p.add_argument("--clear-dir", default=None, help="깨끗한 프레임 디렉터리 (없으면 절차 생성)")
    p.add_argument("--depth-dir", default=None, help="정합된 depth 디렉터리 (선택, 강력 권장)")
    p.add_argument("--val-dir", default=None, help="사전 생성 검증셋 root (hazy/ clear/)")
    p.add_argument("--samples-per-epoch", type=int, default=3000)
    p.add_argument("--patch", type=int, default=240)
    # 학습
    p.add_argument("--epochs", type=int, default=30)
    p.add_argument("--batch-size", type=int, default=16)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--optimizer", choices=("adam", "sgd"), default="adam",
                   help="sgd = 논문 설정(lr 1e-4, momentum 0.9). adam이 5배쯤 빨리 수렴합니다")
    p.add_argument("--weight-decay", type=float, default=1e-4)
    p.add_argument("--clip", type=float, default=0.1, help="gradient clipping. 0이면 끔 (권장 안 함)")
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--seed", type=int, default=0)
    # 손실
    p.add_argument("--fire-prob", type=float, default=0.0,
                   help="깨끗한 프레임에 화염을 합성해 넣을 확률. 학습 프레임에 불이 "
                        "없으면 망이 화염을 본 적이 없어 추론 시 불씨를 지웁니다")
    p.add_argument("--style-probs", default=None,
                   help="연기 스타일 샘플링 비율 'white,gray,sooty,firelit' "
                        "(기본 0.20,0.40,0.25,0.15). sooty에서 성능이 나쁘면 그 비중을 올리세요")
    p.add_argument("--learn-b", action="store_true",
                   help="복원식의 상수 b를 학습 (파라미터 +1). 어두운 장면이 많으면 켜세요 — model.py 주석 참고")
    p.add_argument("--loss", choices=("mse", "l1", "mse+ssim"), default="mse+ssim")
    p.add_argument("--ssim-weight", type=float, default=0.15)
    # 기타
    p.add_argument("--device", default="auto", help="auto | xpu | cuda | cpu")
    p.add_argument("--out", default="runs/base")
    p.add_argument("--resume", default=None)
    p.add_argument("--val-every", type=int, default=1)
    return p.parse_args()


def make_loss(name: str, ssim_weight: float):
    mse, l1 = nn.MSELoss(), nn.L1Loss()

    def fn(pred, target):
        if name == "mse":
            return mse(pred, target)
        if name == "l1":
            return l1(pred, target)
        # MSE만 쓰면 평균적으로는 맞지만 뿌옇게 수렴합니다(회귀의 평균화).
        # SSIM 항이 구조를 붙잡아줘서 얇은 선(주차선·배관)이 훨씬 살아납니다.
        return mse(pred, target) + ssim_weight * (1.0 - ssim(pred, target))

    return fn


@torch.no_grad()
def evaluate(model, loader, device) -> dict:
    model.eval()
    tot_p = tot_s = n = 0.0
    for hazy, clear in loader:
        hazy, clear = hazy.to(device, non_blocking=True), clear.to(device, non_blocking=True)
        pred = model(hazy)
        bs = hazy.size(0)
        tot_p += psnr(pred, clear).item() * bs
        tot_s += ssim(pred, clear).item() * bs
        n += bs
    model.train()
    return {"psnr": tot_p / max(n, 1), "ssim": tot_s / max(n, 1)}


def main():
    args = build_args()
    torch.manual_seed(args.seed)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "args.json").write_text(json.dumps(vars(args), indent=2, ensure_ascii=False))

    device = pick_device(args.device)
    print(f"device : {device} ({device_name(device)})")

    probs = None
    if args.style_probs:
        probs = tuple(float(v) for v in args.style_probs.split(","))
        if len(probs) != 4:
            raise SystemExit("--style-probs 는 값 4개여야 합니다 (white,gray,sooty,firelit)")
    cfg = SmokeConfig(style_probs=probs, fire_prob=args.fire_prob)
    if probs:
        print(f"style : white/gray/sooty/firelit = {probs}")
    if args.fire_prob > 0:
        print(f"fire  : 화염 합성 확률 {args.fire_prob}")
    train_ds = SyntheticSmokeDataset(
        clear_dir=args.clear_dir, depth_dir=args.depth_dir,
        length=args.samples_per_epoch, patch=args.patch, cfg=cfg, seed=None)

    # persistent_workers: 에폭마다 워커를 새로 띄우면 이미지 캐시가 날아가
    # 첫 배치가 매번 느려집니다.
    train_loader = DataLoader(
        train_ds, batch_size=args.batch_size, shuffle=False,
        num_workers=args.workers, drop_last=True,
        persistent_workers=args.workers > 0,
        prefetch_factor=4 if args.workers > 0 else None)

    if args.val_dir:
        val_loader = DataLoader(PairedDataset(args.val_dir, patch=args.patch),
                                batch_size=args.batch_size, num_workers=2)
    else:
        # 고정 시드 = 매 에폭 **똑같은** 검증 샘플. 이게 없으면 PSNR이
        # 오르내리는 게 모델 때문인지 데이터 때문인지 알 수 없습니다.
        val_ds = SyntheticSmokeDataset(
            clear_dir=args.clear_dir, depth_dir=args.depth_dir,
            length=256, patch=args.patch, cfg=cfg, seed=12345)
        val_loader = DataLoader(val_ds, batch_size=args.batch_size, num_workers=2)

    model = AODNet(learn_b=args.learn_b).to(device)
    print(f"params : {count_parameters(model):,}"
          f"{'  (b 학습)' if args.learn_b else ''}")

    # ★ b에는 weight decay를 걸면 안 됩니다.
    #   decay는 "0에 가까울수록 좋다"는 사전지식인데, b는 복원의 기준점(백색점)이라
    #   0으로 끌면 K = (J-b)/(I-1) 의 기준이 무너집니다. 파라미터 하나짜리라
    #   손실에는 거의 안 보이지만 결과는 눈에 띄게 나빠집니다.
    decay_params = [p for n, p in model.named_parameters() if n != "b"]
    groups = [{"params": decay_params, "weight_decay": args.weight_decay}]
    if args.learn_b:
        groups.append({"params": [model.b], "weight_decay": 0.0})

    if args.optimizer == "adam":
        optim = torch.optim.Adam(groups, lr=args.lr)
    else:
        optim = torch.optim.SGD(groups, lr=args.lr, momentum=0.9)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(optim, T_max=args.epochs)
    criterion = make_loss(args.loss, args.ssim_weight)

    start_epoch, best = 0, -1e9
    if args.resume:
        ck = torch.load(args.resume, map_location=device)
        model.load_state_dict(ck["model"])
        optim.load_state_dict(ck["optim"])
        start_epoch, best = ck["epoch"] + 1, ck.get("best", -1e9)
        print(f"resume : {args.resume} (epoch {start_epoch}부터)")

    log_path = out / "log.csv"
    new_log = not log_path.exists()
    log_file = log_path.open("a", newline="")
    writer = csv.writer(log_file)
    if new_log:
        writer.writerow(["epoch", "train_loss", "val_psnr", "val_ssim", "lr", "sec"])

    for epoch in range(start_epoch, args.epochs):
        t0 = time.perf_counter()
        run_loss, steps = 0.0, 0

        for hazy, clear in train_loader:
            hazy = hazy.to(device, non_blocking=True)
            clear = clear.to(device, non_blocking=True)

            pred = model(hazy)
            loss = criterion(pred, clear)

            optim.zero_grad(set_to_none=True)
            loss.backward()
            if args.clip > 0:
                nn.utils.clip_grad_norm_(model.parameters(), args.clip)
            optim.step()

            run_loss += loss.item()
            steps += 1

        synchronize(device)
        sched.step()
        train_loss = run_loss / max(steps, 1)

        if not torch.isfinite(torch.tensor(train_loss)):
            # 여기 걸리면 lr이나 clip 설정을 의심하세요. 계속 돌려봐야
            # 전부 검은 이미지를 뱉는 체크포인트만 쌓입니다.
            raise RuntimeError("loss가 발산했습니다(nan/inf). --lr을 낮추고 --clip 0.1을 확인하세요.")

        metrics = {"psnr": float("nan"), "ssim": float("nan")}
        if (epoch + 1) % args.val_every == 0 or epoch == args.epochs - 1:
            metrics = evaluate(model, val_loader, device)

        dt = time.perf_counter() - t0
        lr_now = optim.param_groups[0]["lr"]
        print(f"[{epoch+1:3d}/{args.epochs}] loss {train_loss:.5f} | "
              f"val PSNR {metrics['psnr']:6.2f} dB  SSIM {metrics['ssim']:.4f} | "
              f"lr {lr_now:.2e} | {dt:5.1f}s")
        writer.writerow([epoch + 1, f"{train_loss:.6f}", f"{metrics['psnr']:.4f}",
                         f"{metrics['ssim']:.5f}", f"{lr_now:.3e}", f"{dt:.2f}"])
        log_file.flush()

        ck = {"model": model.state_dict(), "optim": optim.state_dict(),
              "epoch": epoch, "best": best, "args": vars(args)}
        torch.save(ck, out / "last.pt")
        if metrics["psnr"] == metrics["psnr"] and metrics["psnr"] > best:
            best = metrics["psnr"]
            ck["best"] = best
            torch.save(ck, out / "best.pt")

        empty_cache(device)

    log_file.close()
    if args.learn_b:
        print(f"학습된 b = {float(model.b):.4f}  (초기값 1.0)")
    print(f"\n완료. best val PSNR = {best:.2f} dB")
    print(f"체크포인트: {out/'best.pt'}")


if __name__ == "__main__":
    main()
