from __future__ import annotations

import contextlib
import dataclasses
import enum
import functools
import inspect
import os
import pkgutil
import platform
import threading
import time
from typing import Any, Callable, Dict, Iterator, NamedTuple, Optional, Tuple

import numpy as np
import pybullet as p
from dm_robotics.geometry.geometry import Pose

from coffee import ASSETS_PATH
from coffee.utils import geometry_utils


class BodyState(NamedTuple):
    pose: Pose
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray


class ConnectionMode(enum.Enum):
    """PyBullet client connection mode."""

    DIRECT = p.DIRECT
    GUI = p.GUI
    GUI_SERVER = p.GUI_SERVER
    SHARED_MEMORY = p.SHARED_MEMORY
    SHARED_MEMORY_SERVER = p.SHARED_MEMORY_SERVER
    SHARED_MEMORY_GUI = p.SHARED_MEMORY_GUI


@dataclasses.dataclass(frozen=True)
class ClientConfig:
    """PyBullet client configuration."""

    realtime: bool = True
    egl_render: bool = False
    render_shadows: bool = True
    color: Optional[Tuple[int, int, int]] = None
    width: Optional[int] = None
    height: Optional[int] = None
    gravity_constant: float = -9.81
    physics_timestep: float = 1.0 / 240.0


def _gui_options(cfg: ClientConfig) -> str:
    options = ""
    if cfg.height:
        options += f"--height={cfg.height} "
    if cfg.width:
        options += f"--width={cfg.width} "
    if cfg.color:
        options += f"--background_color_red={cfg.color[0]} "
        options += f"--background_color_blue={cfg.color[1]} "
        options += f"--background_color_green={cfg.color[2]} "
    return options


@dataclasses.dataclass
class BulletClient:
    """Encapsulates all the logic needed for managing a PyBullet physics client.

    Adapted from `pybullet_utils.bullet_client.py`.
    """

    mode: ConnectionMode
    config: ClientConfig
    client_id: int
    pid: int
    post_init_setup: bool

    # This will store all the information that was used to load an object into the
    # client.
    _body_cache: Dict[int, Any] = dataclasses.field(default_factory=dict)

    @classmethod
    def create(
        cls,
        mode: ConnectionMode,
        config: ClientConfig,
    ) -> BulletClient:
        options = _gui_options(config)
        pid = os.getpid()

        # If SHARED_MEMORY, try to connect to an existing simulation.
        if mode.value == p.SHARED_MEMORY:
            client_id = p.connect(p.SHARED_MEMORY, options=options)
            if client_id >= 0:
                print(f"Successfully connected to existing client: {client_id}.")
                return cls(mode, config, client_id, pid, False)

        client_id = p.connect(mode.value, options=options)
        if client_id < 0:
            raise Exception("Could not connect to PyBullet server.")
        return cls(mode, config, client_id, pid, True)

    def __post_init__(self) -> None:
        # Thread locks to safely access thread-unsafe attributes.
        self._realtime_lock = threading.Lock()
        self._thread_alive_lock = threading.Lock()

        if self.post_init_setup:
            self._egl_setup()
            self._physics_setup()
            self._simulation_step_setup()
            self._gui_setup()

        self.setAdditionalSearchPath(ASSETS_PATH)

    def __repr__(self) -> str:
        return f"BulletClient(client_id={self.client_id}, pid={self.pid})"

    def __getattr__(self, name: str) -> Callable:
        """Injects the client ID into all method calls."""
        attribute = getattr(p, name)
        if inspect.isbuiltin(attribute):
            attribute = functools.partial(attribute, physicsClientId=self.client_id)
        return attribute

    def _egl_setup(self) -> None:
        # Setup EGL rendering. Only supported on Linux platforms.
        self._egl_render = False
        is_linux = platform.system() == "Linux"
        if self.mode.value == p.DIRECT and is_linux and self.config.egl_render:
            egl = pkgutil.get_loader("eglRenderer")
            if egl:
                p.loadPlugin(
                    egl.get_filename(),  # type: ignore
                    "_eglRendererPlugin",
                    physicsClientId=self.client_id,
                )
            else:
                p.loadPlugin("eglRendererPlugin", physicsClientId=self.client_id)
            self._egl_render = True

    def _physics_setup(self) -> None:
        # Set the gravity vector.
        self.setGravity(0, 0, self.config.gravity_constant)

    def _gui_setup(self) -> None:
        if self.mode == ConnectionMode.GUI:
            # Move the default camera closer to the scene.
            target = self.getDebugVisualizerCamera()[11]
            self.resetDebugVisualizerCamera(
                cameraDistance=1.5,
                cameraYaw=90,
                cameraPitch=-25,
                cameraTargetPosition=target,
            )
            # Clear the GUI panels.
            self.configureDebugVisualizer(self.COV_ENABLE_GUI, False)
            # Enable or disable shadows.
            self.configureDebugVisualizer(
                self.COV_ENABLE_SHADOWS, self.config.render_shadows
            )

    def _simulation_step_setup(self) -> None:
        # Disable file caching and set the fixed physics timestep.
        self.setPhysicsEngineParameter(
            enableFileCaching=0,
            fixedTimeStep=self.config.physics_timestep,
        )

        self.realtime_mode = self.config.realtime
        if self.mode == ConnectionMode.GUI:
            self.setRealTimeSimulation(1 if self.realtime_mode else 0)
        else:
            if self.realtime_mode and not hasattr(self, "_thread"):
                self._thread = threading.Thread(target=self._infinite_step_simulation)
                self._thread.daemon = True
                self._thread.start()

    def _infinite_step_simulation(self) -> None:
        self.thread_alive = True

        while True:
            if self.realtime_mode and self.thread_alive:
                try:
                    self.stepSimulation()
                except p.error as e:
                    print(f"(_infinite_step_simulation) {e}")
                time.sleep(0.001)

    def disconnect(self) -> None:
        """Disconnects the client from the physics server."""
        self.thread_alive = False

        # NOTE(kevin): We need to explicitly use the `p` module here to prevent an
        # infinite recursion.
        try:
            p.disconnect(physicsClientId=self.client_id)
            # print(f"(id:{self.client_id}) successfully disconnected.")
            self.client_id = -1
        except p.error as e:
            print(f"(id:{self.client_id}) {e}")
            pass

    def __del__(self) -> None:
        if self.client_id >= 0 and self.pid == os.getpid():
            self.disconnect()

    def step(self) -> None:
        """Shorter alias to step the simulation."""
        self.stepSimulation()

    def infinite_step(self) -> None:
        """Step the simulation in an infinite loop. Exit if cntrl-C is pressed."""
        try:
            while True:
                self.step()
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass

    @contextlib.contextmanager
    def disable_rendering(self) -> Iterator[None]:
        self.configureDebugVisualizer(self.COV_ENABLE_RENDERING, 0)
        yield
        self.configureDebugVisualizer(self.COV_ENABLE_RENDERING, 1)

    # Convenience methods.

    def load_urdf(
        self,
        filename: str,
        pose: Optional[Pose] = None,
        scaling: float = 1.0,
        **kwargs,
    ) -> int:
        """Loads a URDF file into the physics client."""
        assert scaling > 0.0, "Scaling must be greater than 0."

        if pose is None:
            pose = Pose()
        base_position = pose.position
        base_orientation = geometry_utils.as_quaternion_xyzw(pose.quaternion)

        body_id = self.loadURDF(
            filename,
            basePosition=base_position,
            baseOrientation=base_orientation,
            globalScaling=scaling,
            **kwargs,
        )

        if body_id < 0:
            raise Exception("Could not load URDF file {filename}")

        # Cache.
        self._body_cache[body_id] = {
            "filename": filename,
            "pose": pose,
            "scaling": scaling,
            **kwargs,
        }

        return body_id

    def reset_body_state(
        self,
        body_id: int,
        pose: Pose,
        linear_velocity: Optional[np.ndarray] = None,
        angular_velocity: Optional[np.ndarray] = None,
    ) -> None:
        # Reset base position and orientation.
        self.resetBasePositionAndOrientation(
            bodyUniqueId=body_id,
            posObj=pose.position,
            ornObj=geometry_utils.as_quaternion_xyzw(pose.quaternion),
        )

        # Reset linear and angular velocities.
        if linear_velocity is not None:
            linear_velocity = linear_velocity.tolist()
        if angular_velocity is not None:
            angular_velocity = angular_velocity.tolist()
        self.resetBaseVelocity(
            objectUniqueId=body_id,
            linearVelocity=linear_velocity,
            angularVelocity=angular_velocity,
        )

    def get_body_state(self, body_id) -> BodyState:
        position, quaternion_xyzw = self.getBasePositionAndOrientation(body_id)
        pose = Pose(
            position=position,
            quaternion=geometry_utils.as_quaternion_wxyz(quaternion_xyzw),
        )
        linear_velocity, angular_velocity = self.getBaseVelocity(body_id)
        return BodyState(
            pose=pose,
            linear_velocity=np.asarray(linear_velocity),
            angular_velocity=np.asarray(angular_velocity),
        )

    def remove_body(self, body_id: int) -> None:
        """Removes a body from the physics client."""
        self.removeBody(bodyUniqueId=body_id)

    # Properties.

    @property
    def egl_render(self) -> bool:
        return self._egl_render

    # realtime_mode

    @property
    def realtime_mode(self) -> bool:
        return self._realtime_mode

    @realtime_mode.setter
    def realtime_mode(self, value: bool) -> None:
        with self._realtime_lock:
            self._realtime_mode = value

    @realtime_mode.getter
    def realtime_mode(self) -> bool:
        with self._realtime_lock:
            return self._realtime_mode

    # thread_alive

    @property
    def thread_alive(self) -> bool:
        return self._thread_alive

    @thread_alive.setter
    def thread_alive(self, value: bool) -> None:
        with self._thread_alive_lock:
            self._thread_alive = value

    @thread_alive.getter
    def thread_alive(self) -> bool:
        with self._thread_alive_lock:
            return self._thread_alive
