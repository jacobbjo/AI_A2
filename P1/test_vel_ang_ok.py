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

        # Left larger than right in the third quadrant
        self.assertFalse(vel_ang_ok(math.atan2(-1, -3), math.atan2(-2, -1), math.atan2(-1, -1)))
        self.assertTrue(vel_ang_ok(math.atan2(-1, -3), math.atan2(-2, -1), math.atan2(1, -1)))
        self.assertTrue(vel_ang_ok(math.atan2(-1, -3), math.atan2(-2, -1), math.atan2(-1, 1)))
        self.assertTrue(vel_ang_ok(math.atan2(-1, -3), math.atan2(-2, -1), math.atan2(1, 1)))

        # Right larger then left in the third quadrant
        self.assertTrue(vel_ang_ok(math.atan2(-2, -1), math.atan2(-1, -3), math.atan2(-1, -1)))
        self.assertFalse(vel_ang_ok(math.atan2(-2, -1), math.atan2(-1, -3), math.atan2(1, -1)))
        self.assertFalse(vel_ang_ok(math.atan2(-2, -1), math.atan2(-1, -3), math.atan2(-1, 1)))
        self.assertFalse(vel_ang_ok(math.atan2(-2, -1), math.atan2(-1, -3), math.atan2(1, 1)))

        # Right in second quadrant, left in third quadrant
        self.assertFalse(vel_ang_ok(math.atan2(1, -3), math.atan2(-1, -3), math.atan2(1, -4)))
        self.assertFalse(vel_ang_ok(math.atan2(1, -3), math.atan2(-1, -3), math.atan2(-1, -4)))
        self.assertTrue(vel_ang_ok(math.atan2(1, -3), math.atan2(-1, -3), math.atan2(-3, -1)))
        self.assertTrue(vel_ang_ok(math.atan2(1, -3), math.atan2(-1, -3), math.atan2(-4, 1)))
        self.assertTrue(vel_ang_ok(math.atan2(1, -3), math.atan2(-1, -3), math.atan2(1, 4)))
        self.assertTrue(vel_ang_ok(math.atan2(1, -3), math.atan2(-1, -3), math.atan2(4, -1)))

        # Left in second quadrant, right in third quadrant
        self.assertTrue(vel_ang_ok(math.atan2(-1, -3), math.atan2(1, -3), math.atan2(1, -4)))
        self.assertTrue(vel_ang_ok(math.atan2(-1, -3), math.atan2(1, -3), math.atan2(-1, -4)))
        self.assertFalse(vel_ang_ok(math.atan2(-1, -3), math.atan2(1, -3), math.atan2(-3, -1)))
        self.assertFalse(vel_ang_ok(math.atan2(-1, -3), math.atan2(1, -3), math.atan2(-4, 1)))
        self.assertFalse(vel_ang_ok(math.atan2(-1, -3), math.atan2(1, -3), math.atan2(1, 4)))
        self.assertFalse(vel_ang_ok(math.atan2(-1, -3), math.atan2(1, -3), math.atan2(4, -1)))





