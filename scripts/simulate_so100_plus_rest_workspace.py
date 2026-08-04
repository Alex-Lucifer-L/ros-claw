"""兼容入口：调用独立的 SO-100 Plus 工作空间扫描包。"""

from rosclaw_mini.workspace_scan.so100_plus import (
    GRIPPER_MODEL_RADIANS,
    REFERENCE_JOINT_RADIANS,
    RestCenteredBox,
    batch_tcp_positions,
    build_parser,
    build_simulation_joint_limits,
    centered_axis_values,
    iter_directed_neighbor_edges,
    largest_valid_box_containing_center,
    main,
    path_has_collision,
    pose_has_collision,
    select_grid_bounds,
)


__all__ = [
    "GRIPPER_MODEL_RADIANS",
    "REFERENCE_JOINT_RADIANS",
    "RestCenteredBox",
    "batch_tcp_positions",
    "build_parser",
    "build_simulation_joint_limits",
    "centered_axis_values",
    "iter_directed_neighbor_edges",
    "largest_valid_box_containing_center",
    "main",
    "path_has_collision",
    "pose_has_collision",
    "select_grid_bounds",
]


if __name__ == "__main__":
    raise SystemExit(main())
