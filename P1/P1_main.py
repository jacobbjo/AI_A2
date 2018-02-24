import numpy as np
import math

def get_avoidance_vels(self, agent):
    """ Returns the avoidance velocities"""
    vels = []

    return vels


def vel_ang_ok(right_ang, left_ang, vel_ang):


    if right_ang > left_ang:
        # The velocity need to be larger than left and smaller than right
        return left_ang < vel_ang < right_ang

    if right_ang <= vel_ang <= left_ang:
        return False
    else:
        return True


left = math.atan2(-1, -3) #+ 2*np.pi
right = math.atan2(-2, -1) #+ 2*np.pi
vel = math.atan2(-1, -1) #+ 2*np.pi

print(vel_ang_ok(right, left, vel))
