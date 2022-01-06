"""Effector for PyBullet bodies."""

import numpy as np
from dm_env import specs

from coffee import effector
from coffee.client import BulletClient


class BulletEffector(effector.Effector):
    def __init__(self) -> None:
        pass

    def action_spec(self, pb_client: BulletClient) -> specs.BoundedArray:
        pass

    def set_control(self, pb_client: BulletClient, command: np.ndarray) -> None:
        pass

    def episode_init(
        self,
        pb_client: BulletClient,
        random_state: np.random.RandomState,
    ) -> None:
        pass


def create_action_spec() -> specs.BoundedArray:
    pass
