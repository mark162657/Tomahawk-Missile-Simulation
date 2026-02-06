import math
import numpy as np

from ..control.timer import InternalTimer
from ..terrain.dem_loader import DEMLoader

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

    def process_update(self, sensed_patch: np.ndarray, est_lat: float, est_lon: float, search_size: int=125) -> tuple[float, float, float]:
        """
        We already obtained the normalized sensed patch 7 * 7 grid underneath our missile, now we will
        search for the certain grid size from our tif for pattern and determine where we might have been.

        """
        if not self.is_ready():
            return None, None, None

        self.last_time_update = self.timer.get_current_time()

        center_row, center_col = self.dem_loader.lat_lon_to_pixel(est_lat, est_lon)
        half_search = search_size // 2
        row_start = max(0, center_row - half_search)
        col_start = max(0, center_col - half_search)

        db_search_patch = self.dem_loader.get_search_area(est_lat, est_lon, search_size, normalized=False)
        snsr_patch_height, snsr_patch_width = sensed_patch.shape

        best_correlation = -1.0 # -1.0 ~ 1.0
        best_offset = (0, 0)
        found_match = False

        db_row, db_col = db_search_patch.shape
        for r in range(db_row - snsr_patch_height + 1): # vertical movement
            for c in range(db_col - snsr_patch_width + 1): # horizontal
                sub_patch = db_search_patch[r: r + snsr_patch_height, c : c + snsr_patch_width] # extract 7 * 7 chunk
                norm_sub_patch = self.dem.normalize_patch(sub_patch)

                # Get NCC
                correlation = np.mean(sensed_patch * norm_sub_patch)

                if correlation > best_correlation:
                    best_correlation = correlation
                    found_match = True
                    best_offset = (r + snsr_patch_height // 2, c + snsr_patch_width // 2) # best matched middle pixel

        if found_match and best_correlation > 0.6:
            # Add best_offset (local patch) to the rest of the larger map, to get exact coordinate pixel
            matched_row = row_start + best_offset[0]
            matched_col = col_start + best_offset[1]

            # Turn row/col into lat/lon
            matched_lat, matched_lon = self.dem_loader.pixel_to_lat_lon(matched_row, matched_col)

            return matched_lat, matched_lon, self.get_noise_covariance() # get lat/lon coordinate and also the noise

        return None, None, None

    def get_noise_covariance(self) -> np.ndarray:
        """
        Calculates the noise covariance matrix.
        The noises are represented as a diagonal in the 3D matrix

        """
        return np.diag([
            self.lateral_accuracy ** 2,
            self.lateral_accuracy ** 2,
            self.vertical_accuracy ** 2
        ])

