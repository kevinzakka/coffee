from __future__ import annotations

import dataclasses
from typing import Tuple


@dataclasses.dataclass(frozen=True)
class PybulletGripperConfig:
    joint_names: Tuple[str, ...]
    pinch_site_joint_name: str
    max_torque: float


@dataclasses.dataclass(frozen=True)
class PybulletGripper:
    ...

    def actuate(self, open_length: float) -> None:
        ...

    def open(self) -> None:
        ...

    def close(self) -> None:
        ...
