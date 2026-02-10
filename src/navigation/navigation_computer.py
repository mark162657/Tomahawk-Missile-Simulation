import numpy as np

from kalman_filter import KalmanFilter
from tercom import TERCOM
from gps import GPS
from ins import INS
from src.terrain.coordinates import CoordinateSystem

class NavigationComputer:
    def __init__(self, start_gps: tuple[float, float], gps_freq_hz: int, ins_freq_hz: int, tercom_freq_hz: int):

        init_pos_array = np.array([0.0, 0.0, 0.0]) # x y z (alt)
        init_vel_array = np.array([0.0, 0.0, 0.0]) # assume 0 at 0 sec

        self.gps_freq_hz = 1.0 / gps_freq_hz
        self.ins_freq_hz = 1.0 / ins_freq_hz
        self.tercom_freq_hz = 1.0 / tercom_freq_hz

        self.gps_nav = GPS()
        self.ins_nav = INS(0, 0, 0)
        self.tercom_nav = TERCOM()
        self.KF = KalmanFilter()

    def update(self):
        pass
