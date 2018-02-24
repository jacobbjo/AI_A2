def vel_ang_ok(right_ang, left_ang, vel_ang):
    """ Checks if the velocity is valid given right and left boundaries """


    if right_ang > left_ang:
        # The velocity need to be larger than left and smaller than right
        return left_ang < vel_ang < right_ang

    if right_ang <= vel_ang <= left_ang:
        return False
    else:
        return True

def get_avoidance_vels(self, agent):
    """ Returns the avoidance velocities"""
    vels = []

    return vels
