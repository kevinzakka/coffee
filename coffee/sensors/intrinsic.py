from __future__ import annotations

import dataclasses
import math
from typing import Tuple

import numpy as np


@dataclasses.dataclass(frozen=True)
class Intrinsic:
    """Relates a camera's internal properties to an ideal pinhole camera model.

    An intrinsic matrix, commonly denoted by K, allows you to transform 3D coordinates
    to 2D coordinates on an image plane. All its attributes are expressed in pixels.

    Attributes:
        width: The width of the camera image.
        height: The height of the camera image.
        fx: The focal length in the x direction.
        fy: The focal length in the y direction.
        cx: The offset of the principal point from the top left corner of the image
            frame in the x direction.
        cy: The offset of the principal point from the top left corner of the image
            frame in the y direction.
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @property
    def resolution(self) -> Tuple[int, int]:
        """Returns a (width, height) tuple."""
        return (self.width, self.height)

    @property
    def matrix(self) -> np.ndarray:
        """Returns the 3x3 matrix form of the intrinsic parameters."""
        return np.array(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

    def level(self, level: int) -> Intrinsic:
        """Returns intrinsic parameters at a specified pyramid level."""

        if level == 0:
            return Intrinsic(
                width=self.width,
                height=self.height,
                fx=self.fx,
                fy=self.fy,
                cx=self.cx,
                cy=self.cy,
            )

        scale_factor = math.pow(0.5, level)
        return Intrinsic(
            width=self.width >> level,
            height=self.height >> level,
            fx=self.fx * scale_factor,
            fy=self.fy * scale_factor,
            cx=(self.cx + 0.5) * scale_factor - 0.5,
            cy=(self.cy + 0.5) * scale_factor - 0.5,
        )
