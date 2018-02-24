import numpy as np
import math
from help_functions import *

left = math.atan2(-1, -3)
right = math.atan2(-2, -1)
vel = math.atan2(-1, -1)

print(vel_ang_ok(right, left, vel))

