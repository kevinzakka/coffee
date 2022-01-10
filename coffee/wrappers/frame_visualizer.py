import dataclasses

from coffee import body
from coffee.client import BulletClient


@dataclasses.dataclass(frozen=True)
class FrameVisualizer:
    """Visualize RGB (R:X, G:Y, B:Z) coordinate frames on the links of a body.

    Each frame is drawn with respect to the local frame of the link.
    """

    body: body.Body
    pb_client: BulletClient

    def draw_link_frame(self, link_name: str) -> None:
        """Draw an RGB frame on the link with the specified name.

        Args:
            link_name: The name of the link to draw the frame on.
        """
        froms = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        tos = [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]
        colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

        for lf, lt, lc in zip(froms, tos, colors):
            self.pb_client.addUserDebugLine(
                lineFromXYZ=lf,
                lineToXYZ=lt,
                lineColorRGB=lc,
                parentObjectUniqueId=self.body.body_id,
                parentLinkIndex=self.body.joints.get_joint_index_from_link_name(
                    link_name
                ),
            )

    def draw_all_link_frames(self) -> None:
        """Draw an RGB frame on all the links of the body."""
        for link_name in self.body.joints.link_names:
            self.draw_link_frame(link_name)
