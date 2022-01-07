"""Classes to make testing with pybullet easier."""

import itertools
from typing import Any, Dict, Iterable

import numpy as np
from absl.testing import absltest, parameterized

from coffee.client import BulletClient, ClientConfig, ConnectionMode

# Special rotations for unit testing, expressed as quaternions.
_NO_ROTATION = np.array([1.0, 0.0, 0.0, 0.0])
_NINETY_DEGREES_ABOUT_X = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0])
_NINETY_DEGREES_ABOUT_Y = np.array([np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0])
_NINETY_DEGREES_ABOUT_Z = np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)])
_FORTYFIVE_DEGREES_ABOUT_X = np.array([np.cos(np.pi / 8), np.sin(np.pi / 8), 0.0, 0.0])
_FORTYFIVE_DEGREES_ABOUT_Y = np.array([np.cos(np.pi / 8), 0.0, np.sin(np.pi / 8), 0.0])
_FORTYFIVE_DEGREES_ABOUT_Z = np.array([np.cos(np.pi / 8), 0.0, 0.0, np.sin(np.pi / 8)])

# Special constants.
_PI = np.pi
_TWO_PI = 2 * np.pi
_HALF_PI = np.pi / 2
_INF = np.inf


class BulletMultiDirectTestCase(absltest.TestCase):
    """PyBullet absltest.TestCase that uses a DIRECT connection for each test method.

    Each class fixture creates its own server-client pair in DIRECT mode. Connects upon
    construction and disconnects upon destruction.
    """

    def setUp(self) -> None:
        self.client = BulletClient.create(
            mode=ConnectionMode.DIRECT,
            config=ClientConfig(realtime=False),
        )

    def tearDown(self) -> None:
        del self.client


class BulletMultiDirectParameterizedTestCase(parameterized.TestCase):
    """Like BulletMultiDirectTestCase but for parameterized.TestCase."""

    def setUp(self) -> None:
        self.client = BulletClient.create(
            mode=ConnectionMode.DIRECT,
            config=ClientConfig(realtime=False),
        )

    def tearDown(self) -> None:
        del self.client


def param_product(**param_lists) -> Iterable[Dict[Any, Any]]:
    """Returns the cartesian product over the values of the given parameters."""
    keys, values = zip(*param_lists.items())
    for combination in itertools.product(*values):
        yield dict(zip(keys, combination))
