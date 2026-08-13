"""Run one complete offline simulated grasp without hardware, GUI or network."""

from __future__ import annotations

import argparse
import json

from rosclaw_mini.runtime import build_sim_runtime
from rosclaw_mini.simulation.pipeline import SimulationGraspPipeline
from rosclaw_mini.simulation.strategy import GraspStrategyVersion


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run a headless simulation-only grasp.")
    parser.add_argument("--scene", default="standard", choices=("standard", "multi_object", "randomized", "noisy", "depth_holes", "boundary_reject"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--strategy", choices=tuple(item.value for item in GraspStrategyVersion), default=GraspStrategyVersion.BASELINE_V1.value)
    parser.add_argument("--task", default="抓取红色立方体")
    args = parser.parse_args(argv)
    runtime = build_sim_runtime(scene_name=args.scene, seed=args.seed)
    try:
        result = SimulationGraspPipeline(runtime, runtime.adapter.world).run(
            args.task,
            strategy_version=GraspStrategyVersion(args.strategy),
            allow_one_reobserve=args.strategy == GraspStrategyVersion.HUMANLIKE_V3.value,
        )
        print(json.dumps({"simulation_only": True, **result.__dict__}, ensure_ascii=False, default=str, indent=2))
        return 0 if result.success else 1
    finally:
        runtime.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
