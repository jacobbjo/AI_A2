from unittest import TestCase
from help_functions import *
from agent import Agent
import numpy as np
import math


class TestVel_ang_ok(TestCase):

    def test_get_neighbors(self):
        v_max = 1.5
        agent_a = Agent(np.array([1, 1], np.array([5, 5], np.array([0.5, 0.5]))))
        agent_b = Agent(np.array([3, 3]), np.array([3, 5], np.array([0.3, 0.8])))
        agent_c = Agent(np.array([10, 10], np.array([3, 3], np.array([1, 1]))))
        neighbors =
        self.assertEqual(a)





