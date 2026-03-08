import numpy as np
import math

from src.control.timer import InternalTimer


class INS:
    def __init__(self, init_pos: np.ndarray, init_vel: np.ndarray, init_att: np.ndarray, update_freq: int=100):
        """
        Args:
            init_pos: [x, y, z] in meters
            init_vel: [vx, vy, vz] in m/s
            init_att: [pitch, roll, heading] in radians
            update_freq: time interval for update of position (Hz)
        """
        self.pos = init_pos.astype(float)
        self.vel = init_vel.astype(float)
        self.att = init_att.astype(float)




        # Deal with time
        self.time_interval = 1.0 / update_freq
        self.timer = InternalTimer()
        self.last_update_time = -float("inf")











    def update_position(self):
        pass