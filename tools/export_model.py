"""Run once with .venv-export/bin/python tools/export_model.py."""

import argparse
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Download YOLO26n and export its COCO detector to NCNN")
    parser.add_argument("--size", type=int, default=320)
    args = parser.parse_args()
    if args.size < 32 or args.size % 32:
        parser.error("Size must be a positive multiple of 32")
    root = Path(__file__).resolve().parents[1]
    os.environ.setdefault("YOLO_CONFIG_DIR", str(root / ".cache/ultralytics"))
    os.environ.setdefault("MPLCONFIGDIR", str(root / ".cache/matplotlib"))
    Path(os.environ["YOLO_CONFIG_DIR"]).mkdir(parents=True, exist_ok=True)
    Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
    from ultralytics import YOLO
    import torch
    torch.set_num_threads(2)
    directory = root / "models"
    directory.mkdir(exist_ok=True)
    os.chdir(directory)
    model = YOLO("yolo26n.pt")
    model.export(format="ncnn", imgsz=args.size, batch=1, device="cpu", quantize=32)


if __name__ == "__main__":
    main()
