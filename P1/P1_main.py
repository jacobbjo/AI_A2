import numpy as np
import math

left = math.atan2(-1, -3) #+ 2*np.pi
right = math.atan2(-2, -1) #+ 2*np.pi
vel = math.atan2(-1, -1) #+ 2*np.pi

print(vel_ang_ok(right, left, vel))
