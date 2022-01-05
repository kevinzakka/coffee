from __future__ import annotations

import dataclasses
from typing import Optional, Protocol, Tuple

import numpy as np
import pybullet as p
from dm_robotics.geometry import geometry
from dm_robotics.transformations import transformations as tr

from coffee.client import BulletClient
from coffee.hints import Array
from coffee.joints import Joints, LinkState
from coffee.sensors.intrinsic import Intrinsic
from coffee.utils.geometry_utils import as_quaternion_wxyz


@dataclasses.dataclass(frozen=True)
class CameraParams:
    """A container for storing and querying the camera parameters and for computing the
    associated projection matrix used by the underyling OpenGL renderer.

    Attributes:
        width: Width of the camera image.
        height: Height of the camera image.
        fovy: Vertical field of view angle of the camera, in degrees.
        znear: Distance from the origin to the near clipping plane.
        zfar: Distance from the origin to the far clipping plane.
    """

    width: int = 128
    height: int = 128
    fovy: float = 90.0
    znear: float = 0.01
    zfar: float = 10.0

    def __post_init__(self) -> None:
        if self.width < 1:
            raise ValueError("width must be greater than 0")
        if self.height < 1:
            raise ValueError("height must be greater than 0")
        if self.fovy <= 0.0:
            raise ValueError("fovy must be greater than 0")

    @property
    def aspect_ratio(self) -> float:
        """The aspect ratio of the rendered image."""
        return self.width / self.height

    def set_znear(self, znear: float) -> CameraParams:
        """Returns a copy of the parameters with the new near value."""
        return dataclasses.replace(self, znear=znear)

    def set_zfar(self, zfar: float) -> CameraParams:
        """Returns a copy of the parameters with the new far value."""
        return dataclasses.replace(self, zfar=zfar)

    @staticmethod
    def from_intrinsic(
        intr: Intrinsic,
        znear: float = 0.01,
        zfar: float = 10.0,
    ) -> CameraParams:
        """Construct the parameters from an intrinsic matrix."""
        fovy = 2 * np.arctan(intr.height / 2 / intr.fy)
        fovy = np.degrees(fovy)
        return CameraParams(
            width=intr.width,
            height=intr.height,
            fovy=fovy,
            znear=znear,
            zfar=zfar,
        )

    def as_projection_matrix(self) -> np.ndarray:
        """Compute the OpenGL perspective projection matrix."""
        K = p.computeProjectionMatrixFOV(
            fov=self.fovy,
            aspect=self.aspect_ratio,
            nearVal=self.znear,
            farVal=self.zfar,
        )
        return np.asarray(K, dtype=np.float32).reshape(4, 4)


class CameraPose(Protocol):
    """A container for querying and storing the pose of the camera in the world frame,
    and for computing the associated view matrix used by the underyling OpenGL renderer.

    All CameraPose classes must have a pose attribute, specifying the 6-DOF pose of the
    camera in the world frame. Additionally, they must implement a `as_view_matrix`
    method which computes the OpenGL view matrix.

    Attributes:
        pose: The camera pose in the world frame.
    """

    @property
    def pose(self) -> geometry.Pose:
        """The camera pose in the world frame."""
        return self.pose

    def as_view_matrix(self) -> np.ndarray:
        """Compute the OpenGL view matrix."""
        ...


@dataclasses.dataclass(frozen=True)
class RPYCameraPose:
    """A `CameraPose` whose orientation is specified via roll-pitch-yaw angles.

    Attributes:
        pose: The pose of the camera in the world frame.
        at: The direction the camera is looking.
        up: The direction of the up vector.
    """

    pose: geometry.Pose
    at: np.ndarray = np.array([0, 0, -1.0])
    up: np.ndarray = np.array([0, 1.0, 0])

    def as_view_matrix(self) -> np.ndarray:
        """Compute the OpenGL view matrix."""
        viewmat = p.computeViewMatrix(
            cameraEyePosition=self.pose.position,
            cameraTargetPosition=self.pose.position + self.rmat @ self.at,
            cameraUpVector=self.rmat @ self.up,
        )
        return np.array(viewmat, dtype=np.float32).reshape(4, 4)

    @staticmethod
    def from_posrpy(
        pos: Array = (0.0, 0.0, 0.0),
        rpy: Array = (0.0, 0.0, 0.0),
        at: Array = (0.0, 0.0, -1.0),
        up: Array = (0.0, 1.0, 0.0),
    ) -> RPYCameraPose:
        """Construct the pose from position and Roll-Pitch-Yaw angles."""
        quat = tr.euler_to_quat(rpy[::-1], ordering="ZYX")
        pose = geometry.Pose(position=pos, quaternion=quat)
        at = np.array(at)
        up = np.array(up)
        return RPYCameraPose(pose=pose, at=at, up=up)

    def as_posrpy(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return the pose as position and Roll-Pitch-Yaw angles."""
        rpy = tr.quat_to_euler(self.pose.quaternion, "ZYX")[::-1]
        return self.pose.position, rpy

    def set_at(self, at: Array) -> RPYCameraPose:
        """Returns a copy of the pose with the provided `at` vector."""
        return dataclasses.replace(self, at=at)

    def set_up(self, up: Array) -> RPYCameraPose:
        """Returns a copy of the pose with the provided `up` vector."""
        return dataclasses.replace(self, up=up)

    @property
    def rmat(self) -> np.ndarray:
        """Return the pose as a 3x3 rotation matrix."""
        return tr.quat_to_mat(self.pose.quaternion)[:3, :3]


@dataclasses.dataclass(frozen=True)
class URDFCameraPose:
    """A `CameraPose` whose pose is hardcoded in a URDF file in the form of a link.

    The URDF file must contain a link with the name `camera_link_name`.

    Attributes:
        pose: The pose of the camera in the world frame.
        pb_client: The `BulletClient` instance.
        body_id: The id of the body containing the camera link.
        camera_link_name: The name of the camera link.
    """

    pb_client: BulletClient
    body_id: int
    camera_link_name: str

    # Initialized in __post_init__.
    _joints: Joints = dataclasses.field(init=False)

    def __post_init__(self) -> None:
        joints = Joints.from_body_id(self.body_id, self.pb_client)
        if not joints.contains_link(self.camera_link_name):
            raise ValueError(f"{self.camera_link_name} does not exist.")
        object.__setattr__(self, "_joints", joints)

    @property
    def pose(self) -> geometry.Pose:
        """The pose of the camera in the world frame."""
        camera_link_state = LinkState(
            *self.pb_client.getLinkState(
                bodyUniqueId=self.body_id,
                linkIndex=self._joints.get_joint_index_from_link_name(
                    self.camera_link_name
                ),
                computeLinkVelocity=0,
                computeForwardKinematics=1,
            )
        )
        position = camera_link_state.link_world_position
        quat = as_quaternion_wxyz(camera_link_state.link_world_orientation)
        return geometry.Pose(position=position, quaternion=quat)

    def as_view_matrix(self) -> np.ndarray:
        """Compute the OpenGL view matrix."""
        hmat = self.pose.hmat
        rmat = hmat[:3, :3]
        cam_rot_y, cam_rot_z = rmat[:, 1], rmat[:, 2]
        eye_pos = hmat[:3, 3]
        target_pos = eye_pos + cam_rot_y
        up_vec = cam_rot_z
        viewmat = self.pb_client.computeViewMatrix(
            cameraEyePosition=eye_pos,
            cameraTargetPosition=target_pos,
            cameraUpVector=up_vec,
        )
        return np.array(viewmat, dtype=np.float32).reshape(4, 4)


@dataclasses.dataclass(frozen=True)
class RelativeToCameraPose:
    """A `CameraPose` whose pose is specified relative to a PyBullet body.

    Attributes:
        pose: The pose of the camera in the world frame.
        pb_client: The `BulletClient` instance.
        body_id: The id of the body containing the camera link.
        relative_pose: The pose of the camera relative to the link.
        link_name: The name of the link the pose is defined relative to.
    """

    pb_client: BulletClient
    body_id: int
    relative_pose: geometry.Pose
    link_name: str

    # Initialized in __post_init__.
    _joints: Joints = dataclasses.field(init=False)
    _link_id: int = dataclasses.field(init=False)

    def __post_init__(self) -> None:
        joints = Joints.from_body_id(self.body_id, self.pb_client)
        link_id = joints.get_joint_index_from_link_name(self.link_name)
        object.__setattr__(self, "_joints", joints)
        object.__setattr__(self, "_link_id", link_id)

    @property
    def pose(self) -> geometry.Pose:
        """The pose of the camera in the world frame."""
        link_state = LinkState(
            *self.pb_client.getLinkState(
                bodyUniqueId=self.body_id,
                linkIndex=self._link_id,
                computeLinkVelocity=0,
                computeForwardKinematics=1,
            )
        )
        position = link_state.link_world_position
        quat = as_quaternion_wxyz(link_state.link_world_orientation)
        link_pose = geometry.Pose(position=position, quaternion=quat)
        return link_pose.mul(self.relative_pose)

    def as_view_matrix(self) -> np.ndarray:
        """Compute the OpenGL view matrix."""
        hmat = self.pose.hmat
        rmat = hmat[:3, :3]
        cam_rot_y, cam_rot_z = rmat[:, 1], rmat[:, 2]
        eye_pos = hmat[:3, 3]
        target_pos = eye_pos + cam_rot_y
        up_vec = cam_rot_z
        viewmat = self.pb_client.computeViewMatrix(
            cameraEyePosition=eye_pos,
            cameraTargetPosition=target_pos,
            cameraUpVector=up_vec,
        )
        return np.array(viewmat, dtype=np.float32).reshape(4, 4)


@dataclasses.dataclass(frozen=True)
class CameraFrame:
    """A rendered frame from a Camera.

    Attributes:
        color: The rendered RGB image.
        depth: The rendered depth image if depth was enabled, else None.
        segmentation: The rendered segmentation mask if segmentation was enabled, else
            None.
    """

    color: np.ndarray
    depth: Optional[np.ndarray] = None
    segmentation: Optional[np.ndarray] = None


@dataclasses.dataclass(frozen=True)
class Camera:
    """A PyBullet camera."""

    pb_client: BulletClient
    params: CameraParams
    cam_pose: CameraPose

    # Initialized in __post_init__.
    viewmat: np.ndarray = dataclasses.field(init=False)
    projmat: np.ndarray = dataclasses.field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "viewmat", self.cam_pose.as_view_matrix())
        object.__setattr__(self, "projmat", self.params.as_projection_matrix())

    def _pre_render(self) -> None:
        ...

    def _post_render(self) -> None:
        ...

    def _render(
        self,
        depth: bool = False,
        segmentation: bool = False,
    ) -> CameraFrame:
        depth_im, segm_im = None, None

        if segmentation:
            flags = self.pb_client.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX
        else:
            flags = self.pb_client.ER_NO_SEGMENTATION_MASK

        if self.pb_client.egl_render:
            renderer = self.pb_client.ER_BULLET_HARDWARE_OPENGL
        else:
            renderer = self.pb_client.ER_TINY_RENDERER

        _, _, color_pix, depth_pix, segm_pix = self.pb_client.getCameraImage(
            width=self.params.width,
            height=self.params.height,
            # NOTE(kevin): pybullet API expects flattened view and projection matrices.
            viewMatrix=self.viewmat.ravel(),
            projectionMatrix=self.projmat.ravel(),
            shadow=self.pb_client.config.render_shadows,
            flags=flags,
            renderer=renderer,
        )

        # Get color image.
        color_shape = (self.params.height, self.params.width, 4)
        color_im = np.array(color_pix, dtype=np.uint8).reshape(color_shape)
        color_im = color_im[:, :, :3]  # Eliminate alpha channel.

        if depth:
            # Get depth image by converting non-linear z-buffer to depth in meters, see:
            # http://stackoverflow.com/a/6657284/1461210
            depth_shape = (self.params.height, self.params.width)
            znear, zfar = self.params.znear, self.params.zfar
            z_const = 2.0 * znear * zfar
            zfmzn = zfar - znear
            zfpzn = zfar + znear
            z_buffer = np.array(depth_pix).reshape(depth_shape)
            z_n = 2.0 * z_buffer - 1.0  # [-1, 1] to [0, 1].
            depth_im = z_const / (zfpzn - z_n * zfmzn)

        if segmentation:
            segm_im = np.array(segm_pix, dtype=np.uint32).reshape(depth_shape)

        return CameraFrame(color_im, depth_im, segm_im)

    # Public methods.

    def render(
        self,
        depth: bool = False,
        segmentation: bool = False,
    ) -> CameraFrame:
        self._pre_render()
        frame = self._render(depth, segmentation)
        self._post_render()
        return frame


@dataclasses.dataclass(frozen=True)
class MoveableCamera(Camera):
    """A `Camera` that can be moved by changing its pose.

    Since `Camera` objects are immutable, this class returns a new MoveableCamera object
    with an updated `cam_pose`.
    """

    cam_pose: RPYCameraPose

    # Public methods.

    def set_roll(self, roll: float) -> MoveableCamera:
        cur_pos, cur_rpy = self.cam_pose.as_posrpy()
        cur_rpy[0] = roll
        return self.set_pose_from_posrpy(cur_pos, cur_rpy)

    def set_pitch(self, pitch: float) -> MoveableCamera:
        cur_pos, cur_rpy = self.cam_pose.as_posrpy()
        cur_rpy[1] = pitch
        return self.set_pose_from_posrpy(cur_pos, cur_rpy)

    def set_yaw(self, yaw: float) -> MoveableCamera:
        cur_pos, cur_rpy = self.cam_pose.as_posrpy()
        cur_rpy[2] = yaw
        return self.set_pose_from_posrpy(cur_pos, cur_rpy)

    def set_position(self, position: Array) -> MoveableCamera:
        _, cur_rpy = self.cam_pose.as_posrpy()
        return self.set_pose_from_posrpy(position, cur_rpy)

    def set_pose_from_posrpy(self, pos: Array, rpy: Array) -> MoveableCamera:
        cam_pose = RPYCameraPose.from_posrpy(pos, rpy)
        return dataclasses.replace(self, cam_pose=cam_pose)


@dataclasses.dataclass(frozen=True)
class URDFCamera(Camera):
    """A `Camera` explicitly defined in a URDF.

    Unlike `MovableCamera`, this camera's pose is fixed and cannot be explicitly
    changed. However, its pose is constantly updated at every simulation step, and if
    the camera is attached to a moving body, it will move.

    Such a camera can be useful for explicitly attaching a camera to a robot's end
    effector for example.
    """

    cam_pose: URDFCameraPose

    def _pre_render(self) -> None:
        object.__setattr__(self, "viewmat", self.cam_pose.as_view_matrix())


@dataclasses.dataclass(frozen=True)
class RelativeToCamera(Camera):
    """A `Camera` whose pose is defined relative to a PyBullet body.

    Similar to `URDFCamera` but the relative pose is defined in code rather than in
    the URDF.
    """

    cam_pose: RelativeToCameraPose

    def _pre_render(self) -> None:
        object.__setattr__(self, "viewmat", self.cam_pose.as_view_matrix())


@dataclasses.dataclass(frozen=True)
class TrackingCamera:
    # TODO(kevin): Reminder to implement this.
    ...
