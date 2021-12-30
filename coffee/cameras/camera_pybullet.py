import dataclasses
from typing import Optional, Tuple

import numpy as np

from coffee.cameras.camera import CameraFrame
from coffee.client import BulletClient
from coffee.lie import SE3
from coffee.manipulators.manipulator import Manipulator


@dataclasses.dataclass(frozen=True)
class Intrinsic:
    """Pinhole camera parameters."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @property
    def matrix(self) -> np.ndarray:
        return np.array(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )


@dataclasses.dataclass(frozen=True)
class PybulletCameraConfig:
    image_size: Tuple[int, int]
    intrinsics: Intrinsic
    pose: SE3
    zrange: Tuple[float, float]
    noise: Optional[float] = None
    segmentation: bool = False


@dataclasses.dataclass(frozen=False)
class PybulletCamera:
    """A camera in PyBullet."""

    pb_client: BulletClient
    camera_config: PybulletCameraConfig

    def _compute_view_matrix(self) -> None:
        self.view_matrix = self.pb_client.computeViewMatrix(
            cameraEyePosition=self.camera_config.pose.translation,
            cameraTargetPosition=self.camera_config.pose.translation + self.lookdir,
            cameraUpVector=self.updir,
        )

    def _compute_projection_matrix(self) -> None:
        self.projection_matrix = self.pb_client.computeProjectionMatrixFOV(
            fov=self.fov_h,
            aspect=self.aspect_ratio,
            nearVal=self.camera_config.zrange[0],
            farVal=self.camera_config.zrange[1],
        )

    def __post_init__(self) -> None:
        self.height, self.width = self.camera_config.image_size
        self.aspect_ratio = float(self.width / self.height)

        # OpenGL camera settings.
        rotm = self.camera_config.pose.rotation.as_matrix()
        # rotm = (
        #     np.asarray(
        #         self.pb_client.getMatrixFromQuaternion(
        #             self.camera_config.pose.rotation.xyzw,
        #         )
        #     )
        #     .astype(np.float32)
        #     .reshape(3, 3)
        # )
        self.lookdir = (
            rotm @ np.array([0, 0, 1], dtype=np.float32).reshape(3, 1)
        ).reshape(-1)
        self.updir = (
            rotm @ np.array([0, -1, 0], dtype=np.float32).reshape(3, 1)
        ).reshape(-1)

        # Vertical field of view.
        focal_length = self.camera_config.intrinsics.fx
        fov = (self.height / 2) / focal_length
        self.fov_h = 180 * np.arctan(fov) * 2 / np.pi

        self._compute_view_matrix()
        self._compute_projection_matrix()

    def get_frame(self) -> CameraFrame:
        if self.pb_client.egl_render:
            renderer = self.pb_client.ER_BULLET_HARDWARE_OPENGL
        else:
            renderer = self.pb_client.ER_TINY_RENDERER

        if self.camera_config.segmentation:
            flags = self.pb_client.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX
        else:
            flags = self.pb_client.ER_NO_SEGMENTATION_MASK

        # Render with OpenGL camera settings.
        _, _, color, depth, segm = self.pb_client.getCameraImage(
            width=self.camera_config.image_size[1],
            height=self.camera_config.image_size[0],
            viewMatrix=self.view_matrix,
            projectionMatrix=self.projection_matrix,
            shadow=1,
            flags=flags,
            # Note when use_egl is toggled, this option will not actually use openGL
            # but EGL instead.
            renderer=renderer,
        )

        # Get color image.
        color_image_size = (self.height, self.width, 4)
        color = np.array(color, dtype=np.uint8).reshape(color_image_size)
        color = color[:, :, :3]  # Remove alpha channel.
        # if self.camera_config['noise']:
        #     color = np.int32(color)
        #     color += np.int32(self._random.normal(0, 3, self.camera_config['image_size']))
        #     color = np.uint8(np.clip(color, 0, 255))

        # Get depth image.
        depth_image_size = (self.height, self.width)
        z_buffer = np.array(depth).reshape(depth_image_size)

        z_far, z_near = self.camera_config.zrange
        depth = z_far * z_near / (z_far - (z_far - z_near) * z_buffer)
        # if self.camera_config['noise']:
        #     depth += self._random.normal(0, 0.003, depth_image_size)

        # Get segmentation image.
        if self.camera_config.segmentation:
            segm = np.uint8(segm).reshape(depth_image_size)
        else:
            segm = None

        return CameraFrame(color=color, depth=depth, segmentation=segm)


@dataclasses.dataclass(frozen=False)
class PybulletEndEffectorCamera(PybulletCamera):

    pb_client: BulletClient
    camera_config: PybulletCameraConfig
    manipulator: Manipulator

    def _compute_view_matrix(self) -> None:
        camera_ls = self.pb_client.getLinkState(
            bodyUniqueId=self.manipulator.body_id,
            linkIndex=self.manipulator.joints.name2index["camera_joint"],
        )
        # pose = self.manipulator.get_eef_pose()
        camera_pos, camera_orn = camera_ls[:2]
        cam_rot = self.pb_client.getMatrixFromQuaternion(camera_orn)
        cam_rot = np.array(cam_rot).reshape(3, 3)
        cam_rot_y, cam_rot_z = cam_rot[:, 1], cam_rot[:, 2]

        self.view_matrix = self.pb_client.computeViewMatrix(
            cameraEyePosition=camera_pos,
            cameraTargetPosition=camera_pos + cam_rot_y,
            cameraUpVector=-cam_rot_z,
        )

    def get_frame(self) -> CameraFrame:
        self._compute_view_matrix()
        return super().get_frame()
