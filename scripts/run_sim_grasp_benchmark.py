"""Run repeatable headless simulation-only strategy comparisons."""

from __future__ import annotations

import argparse
from pathlib import Path

from rosclaw_mini.simulation.benchmark import (
    run_headless_benchmark,
    save_benchmark_results,
    summarize_runs,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run offline SO-100 simulation benchmark.")
    parser.add_argument("--output", type=Path, default=Path("artifacts/simulation/benchmark.json"))
    parser.add_argument("--seeds", type=int, default=3)
    args = parser.parse_args(argv)
    if args.seeds <= 0:
        parser.error("--seeds 必须大于 0")
    runs = run_headless_benchmark(seeds=tuple(range(args.seeds)))
    output = save_benchmark_results(args.output, runs)
    for summary in summarize_runs(runs):
        print(summary)
    print(f"simulation-only results: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
