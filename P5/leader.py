import numpy as np

class Leader(object):

    def __init__(self, start_pos, trajectory_goal):
        self.pos = start_pos
        #self.trajectory_start = trajectory_start
        self.goal = trajectory_goal
        self.pos_hist = [self.pos]  # keep track of all positions for later visualization
        self.vel = np.zeros(2)
        self.r = 0.5


    def at_goal(self, the_map):
        """ Returns whether the minion is at its goal or not"""
        if np.linalg.norm(self.goal - self.pos) < the_map.vehicle_v_max * the_map.vehicle_dt:
            return True
        return False

    def at_start(self, the_map):
        """ Returns whether the minion is at its goal or not"""
        if np.linalg.norm(self.trajectory_start - self.pos) < the_map.vehicle_v_max * the_map.vehicle_dt:
            return True
        return False

    def save_pos(self):
        self.pos_hist.append(np.copy(self.pos))

    #def move(self, the_map):
    #
    #    if not np.linalg.norm(self.trajectory_start - self.pos) < the_map.vehicle_v_max * the_map.vehicle_dt:
    #        self.vel = self.trajectory_start - self.pos
    #        self.vel = (self.vel / np.linalg.norm(self.trajectory_start-self.pos)) * the_map.vehicle_v_max
    #        self.pos = self.pos + (self.vel * the_map.vehicle_dt)
    #    self.save_pos()

    def move_trajectory(self, position):

        self.vel = position - self.pos
        self.pos = position
        self.save_pos()