import numpy as np
import math

from ..terrain.dem_loader import DEMLoader
from ..control.timer import InternalTimer
from ..sensors.gps_receiver import GPSReceiver

class GPS:
    def __init__(self, update_freq: int=1):
        """
        Assume (feature might be added in future, depends):
            - signal travel time does not exists + processing delay
            - PPS + WAGE enhancement signal (acheiving around 1 meter accuracy)
            - ignore signal processing layer, solved triangulation and output Cartesian coordinates (x, y, z) directly
        """

        # Time related stuff
        self.time_interval = 1.0 / update_freq # fix amount of time (sec) between each consecutive GPS update
        self.timer = InternalTimer() # setup an InternalTimer class for time-related operations
        self.last_update_time = -float("inf") # the time which last GPS adjustment / fix was performed

        # DEM operation
        dem = DEMLoader()
        self.dem_laoder = dem

        # Check if GPS is active and jammed or not
        self.has_signal = True
        self.is_jammed = False

        # GPS receiver
        self.receiver = GPSReceiver()

    def is_ready(self) -> bool:
        """
        Check if GPS is ready for next measurement by:
            1. Check if GPS has signal
            2. Minus the current time by the last timestamp which we update
                our GPS state, and see if its time interval exceeds our standard fixed time interval
                between each measurement.
        Args:
            - current_time: current time recorded in our InternalTimer system
        """

        if not self.has_signal:
            return False

        current_time = self.timer.get_current_time()

        # Return if time elapsed since last GPS check is greater than the time_interval (see __init__ for definition)
        return (current_time - self.last_update_time) >= self.time_interval

    def get_gps_location(self, location: list[float, float]) -> list[float, float]:
        if not self.is_ready():
            return None, None

        # Pull measurement data from GPS receiver
        raw_measurement = self.receiver.get_raw_measurement(location)

        # if self.detect_jammed(raw_measurement):
        #     self.is_jammed = True
        #     return None

        self.last_update_time = self.timer.get_elapsed_time()
        return raw_measurement

    def detect_jammed(self, measurement: np.ndarray) -> bool:
        """TODO: Future implementation"""
        pass




        


    
    