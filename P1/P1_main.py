import numpy as np
import math
from help_functions import *
from importJSON1 import Problem

left = math.atan2(-1, -3)
right = math.atan2(-2, -1)
vel = math.atan2(-1, -1)

the_map = Problem("source\\P21.json")

vmax = the_map.vehicle_v_max
print(vmax)
#print(vel_ang_ok(right, left, vel))

#neighbor_limit = vmax * dt * 5

# -------- Creates the agents and stores them in list

#for agents in len(map.startPositions)