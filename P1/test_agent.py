from unittest import TestCase
from agent import Agent
import numpy as np
import math


class TestAgent(TestCase):
    agent = Agent(np.array([1, 1], np.array[2, 2]))

    def test_distance_to(self):
        self.fail()

    def test_find_best_vel(self):
        self.fail()

    def test_get_bound_ang(self):
        self.fail()

    def test_get_avoidance_vels(self):
        self.fail()

    def test_vel_ang_ok(self):
        # Left larger then right in the first quadrant
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(1, 1)))
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(-1, 1)))
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(1, -1)))
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(-1, -1)))
        # vel is equal as one of the boundaries
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(1, 2)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(2, 1)))

        # Left smaller then right in the first quadrant
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(2, 1), math.atan2(1, 2), math.atan2(1, 1)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(2, 1), math.atan2(1, 2), math.atan2(-1, 1)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(2, 1), math.atan2(1, 2), math.atan2(1, -1)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(2, 1), math.atan2(1, 2), math.atan2(-1, -1)))
        # vel is equal as one of the boundaries
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(1, 2)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(1, 2), math.atan2(2, 1), math.atan2(2, 1)))

        # Left larger than right in the third quadrant
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(-1, -3), math.atan2(-2, -1), math.atan2(-1, -1)))
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(-1, -3), math.atan2(-2, -1), math.atan2(1, -1)))
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(-1, -3), math.atan2(-2, -1), math.atan2(-1, 1)))
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(-1, -3), math.atan2(-2, -1), math.atan2(1, 1)))

        # Right larger then left in the third quadrant
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(-2, -1), math.atan2(-1, -3), math.atan2(-1, -1)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(-2, -1), math.atan2(-1, -3), math.atan2(1, -1)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(-2, -1), math.atan2(-1, -3), math.atan2(-1, 1)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(-2, -1), math.atan2(-1, -3), math.atan2(1, 1)))

        # Right in second quadrant, left in third quadrant
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(1, -3), math.atan2(-1, -3), math.atan2(1, -4)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(1, -3), math.atan2(-1, -3), math.atan2(-1, -4)))
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(1, -3), math.atan2(-1, -3), math.atan2(-3, -1)))
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(1, -3), math.atan2(-1, -3), math.atan2(-4, 1)))
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(1, -3), math.atan2(-1, -3), math.atan2(1, 4)))
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(1, -3), math.atan2(-1, -3), math.atan2(4, -1)))

        # Left in second quadrant, right in third quadrant
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(-1, -3), math.atan2(1, -3), math.atan2(1, -4)))
        self.assertTrue(self.agent.vel_ang_ok(math.atan2(-1, -3), math.atan2(1, -3), math.atan2(-1, -4)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(-1, -3), math.atan2(1, -3), math.atan2(-3, -1)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(-1, -3), math.atan2(1, -3), math.atan2(-4, 1)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(-1, -3), math.atan2(1, -3), math.atan2(1, 4)))
        self.assertFalse(self.agent.vel_ang_ok(math.atan2(-1, -3), math.atan2(1, -3), math.atan2(4, -1)))
