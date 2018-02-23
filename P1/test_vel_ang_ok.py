from unittest import TestCase
from P1_main import vel_ang_ok
import math


class TestVel_ang_ok(TestCase):
    def test_vel_ang_ok(self):
        # Left larger then right in the first quadrant
        self.assertFalse(vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(1, 1)))
        self.assertTrue(vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(-1, 1)))
        self.assertTrue(vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(1, -1)))
        self.assertTrue(vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(-1, -1)))
        # vel is equal as one of the boundaries
        self.assertFalse(vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(1, 2)))
        self.assertFalse(vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(2, 1)))

        # Left smaller then right in the first quadrant
        self.assertTrue(vel_ang_ok(math.atan2(2, 1), math.atan2(1, 2), math.atan2(1, 1)))
        self.assertFalse(vel_ang_ok(math.atan2(2, 1), math.atan2(1, 2), math.atan2(-1, 1)))
        self.assertFalse(vel_ang_ok(math.atan2(2, 1), math.atan2(1, 2), math.atan2(1, -1)))
        self.assertFalse(vel_ang_ok(math.atan2(2, 1), math.atan2(1, 2), math.atan2(-1, -1)))
        # vel is equal as one of the boundaries
        self.assertFalse(vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(1, 2)))
        self.assertFalse(vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(2, 1)))


