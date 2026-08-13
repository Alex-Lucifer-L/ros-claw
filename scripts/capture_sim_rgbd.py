"""Save one virtual RGB frame and depth visualization; no GUI or device."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from rosclaw_mini.runtime import build_sim_runtime
from rosclaw_mini.simulation.camera import VirtualRGBDCamera


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Capture one simulation-only RGB-D frame.")
    parser.add_argument("--scene", default="standard", choices=("standard", "multi_object", "randomized", "noisy", "depth_holes", "boundary_reject"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/simulation/frames"))
    args = parser.parse_args(argv)
    runtime = build_sim_runtime(scene_name=args.scene, seed=args.seed)
    try:
        with VirtualRGBDCamera(runtime.adapter.world) as camera:
            frame = camera.capture_frame()
        args.output_dir.mkdir(parents=True, exist_ok=True)
        rgb_path = args.output_dir / "sim_rgb.png"
        depth_path = args.output_dir / "sim_depth.png"
        cv2.imwrite(str(rgb_path), cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR))
        depth_m = frame.aligned_depth.astype(float) * frame.depth_scale_m_per_unit
        nonzero = depth_m[depth_m > 0.0]
        if nonzero.size:
            maximum = float(np.percentile(nonzero, 99))
            view = np.clip(depth_m / max(maximum, 1e-6) * 255.0, 0, 255).astype(np.uint8)
        else:
            view = np.zeros_like(frame.aligned_depth, dtype=np.uint8)
        cv2.imwrite(str(depth_path), cv2.applyColorMap(view, cv2.COLORMAP_TURBO))
        print(f"simulation-only RGB: {rgb_path}")
        print(f"simulation-only depth: {depth_path}")
        return 0
    finally:
        runtime.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
