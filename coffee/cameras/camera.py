import dataclasses
from typing import Optional, Protocol

import numpy as np


@dataclasses.dataclass(frozen=True)
class CameraFrame:
    """Holds camera frame data."""

    color: np.ndarray
    depth: np.ndarray
    segmentation: Optional[np.ndarray] = None


class Camera(Protocol):
    """Functionality that needs to be implemented by all cameras."""

    def get_frame(self) -> CameraFrame:
        """Captures a frame from the camera."""
        ...
