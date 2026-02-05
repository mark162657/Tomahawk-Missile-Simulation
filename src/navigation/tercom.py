import math
import numpy as np

from ..control.timer import InternalTimer
from ..terrain.dem_loader import DEMLoader
from ..control import *

class TERCOM:
    """
    TERCOM (Terrain Contour Matching): A navigation system that guides missiles by
    comparing ground elevation profiles measured by radar altimeter with pre-stored
    terrain maps to determine position and correct flight path. Implemented by Kalman Filter.

    The terrain check will be performed every 5 seconds. With following data:
        - db data: from TerrainDatabase.get_elevation_patch
        - sensor data:
    """
    def __init__(self, location: list[float, float], update_freq: int=10):
        """
        Args:
            - location: receive the current location of the missile, not a true absolute location
            - update_freq: update the time interval (Hz)
        """
        self.location = location

        dem = DEMLoader()
        self.dem_loader = dem

        # Get location patch
        self.location_pixel = self.dem_loader.lat_lon_to_pixel(self.location) # first by turning lat/lon to pixel
        self.location_patch = self.dem_loader.get_location_patch(self.location_pixel) # get a patch under the missile

        # Deal with accuracy
        self.lateral_accuracy = 12.0  # meters
        self.vertical_accuracy = 2.5  # meters

        # Time related operation (same as gps.py)
        self.time_interval = 1.0 / update_freq
        self.timer = InternalTimer()
        self.last_update_time = -float("inf")

    def is_ready(self) -> bool:
        """
        Check if TERCOM navigation is ready to work, by checking whether it is the right time to work
        """
        current_time = self.timer.get_current_time()
        return (current_time - self.last_update_time) >= self.time_interval

    def process_update(self, sensed_patch, est_lat, est_lon, search_size: int=90):
        """
        We already obtained the normalized sensed patch 7 * 7 grid underneath our missile, now we will
        search for the certain grid size from our tif for pattern and determine where we might have been.

        """
        if not self.is_ready():
            return None, None

        db_search_area = self.dem_loader.get_search_area(est_lat, est_lon, search_size)

        if db_search_area is None or db_search_area[0] < 7 or db_search_area[1] < 7:
            return None, None, None

        self.last_time_update = self.timer.get_current_time()




















