class Minion(object):

    def __init__(self, start_pos, traject_start, leader_traject_start):
        self.start_pos = start_pos
        self.traject_start = traject_start
        self.offset_x = leader_traject_start[0] - traject_start[0]
        self.offset_y = leader_traject_start[1] - traject_start[1]
