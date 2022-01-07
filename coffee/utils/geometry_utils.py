from typing import Optional

import numpy as np

from coffee.hints import Array


def l2_normalize(
    x: np.ndarray,
    axis: Optional[int] = None,
    epsilon: float = 1e-12,
) -> np.ndarray:
    """L2 normalize an array with numerical stability."""
    square_sum = np.sum(np.square(x), axis=axis, keepdims=True)  # type: ignore
    x_inv_norm = 1.0 / np.sqrt(np.maximum(square_sum, epsilon))
    return x * x_inv_norm


def as_quaternion_xyzw(wxyz: Array) -> np.ndarray:
    """Convert a quaternion from WXYZ to XYZW format."""
    return np.roll(wxyz, shift=-1)


def as_quaternion_wxyz(xyzw: Array) -> np.ndarray:
    """Convert a quaternion from XYZW to WXYZ format."""
    return np.roll(xyzw, shift=1)


def quaternion_equal(actual: Array, expected: Array) -> bool:
    """Returns True if two quaternions are equal."""
    actual = np.asarray(actual)
    expected = np.asarray(expected)
    return np.allclose(actual, expected) or np.allclose(actual, expected * -1)
