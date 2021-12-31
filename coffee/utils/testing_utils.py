"""Classes to make testing with pybullet easier."""

from absl.testing import absltest, parameterized

from coffee.client import BulletClient, ClientConfig, ConnectionMode


class BulletMultiDirectTestCase(absltest.TestCase):
    """PyBullet absltest.TestCase that uses a DIRECT connection for each test method.

    A BulletClient is created before the test method is run and destroyed immediately
    afte

    Each class fixture creates its own server-client pair in DIRECT mode. Connects upon
    construction and disconnects upon destruction.
    """

    def setUp(self) -> None:
        self.client = BulletClient.create(
            mode=ConnectionMode.DIRECT,
            config=ClientConfig(realtime=False, render_shadows=False),
        )

    def tearDown(self) -> None:
        del self.client


class BulletMultiDirectParameterizedTestCase(parameterized.TestCase):
    """Like BulletMultiDirectTestCase but for parameterized.TestCase."""

    def setUp(self) -> None:
        self.client = BulletClient.create(
            mode=ConnectionMode.DIRECT,
            config=ClientConfig(realtime=False, render_shadows=False),
        )

    def tearDown(self) -> None:
        del self.client
