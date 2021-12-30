import dataclasses
from typing import Protocol, Tuple


@dataclasses.dataclass(frozen=True)
class EndEffectorConfig:
    joint_names: Tuple[str, ...]


class EndEffector(Protocol):
    """Functionality that needs to be implemented by all end effectors."""

    ...
