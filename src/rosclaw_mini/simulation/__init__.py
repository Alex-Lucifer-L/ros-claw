"""Headless, explicitly simulation-only SO-100 Plus research environment.

Nothing in this package opens a serial device, a camera device, or a network
connection.  It is deliberately separate from the ``so100_plus`` production
backend so an installed device can never make a simulation command physical.
"""

from rosclaw_mini.simulation.config import (
    SimulationCameraConfig,
    SimulationObjectSpec,
    SimulationSceneConfig,
    build_simulation_scene,
    load_simulation_camera_config,
)

__all__ = (
    "SimulationCameraConfig",
    "SimulationObjectSpec",
    "SimulationSceneConfig",
    "build_simulation_scene",
    "load_simulation_camera_config",
)
