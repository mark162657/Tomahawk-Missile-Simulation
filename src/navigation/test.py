import numpy as np
import math
from ..terrain.dem_loader import DEMLoader
from ..control.timer import InternalTimer


class TERCOM_Test:
    """
    TERCOM (Terrain Contour Matching): Correlates sensed terrain patches
    against a stored DEM to provide absolute position updates.
    """

    def __init__(self, timer, dem_path, update_freq=0.2):
        """
        Args:
            timer: Reference to the central InternalTimer for synchronization.
            dem_path: Path to the .tif DEM file.
            update_freq: Frequency of terrain checks (Hz). Default 0.2 (every 5 seconds).
        """
        self.dem_loader = DEMLoader(dem_path)
        self.timer = timer

        # Performance specs (matching KalmanFilter's R_TERCOM)
        self.lateral_accuracy = 12.0  # meters
        self.vertical_accuracy = 2.5  # meters

        # Timing
        self.time_interval = 1.0 / update_freq
        self.last_update_time = -float("inf")

    def is_ready(self) -> bool:
        current_time = self.timer.get_current_time()
        return (current_time - self.last_update_time) >= self.time_interval

    def process_update(self, sensed_patch: np.ndarray, est_lat: float, est_lon: float, search_radius_px: int = 125):
        """
        Performs the Terrain Contour Match.

        Args:
            sensed_patch: A 7x7 normalized elevation patch from Radar/LiDAR sensors.
            est_lat, est_lon: The current estimated position from the INS.
            search_radius_px: Pixels to search in each direction (~125px = ~250x250 search area).

        Returns:
            Tuple (measured_lat, measured_lon, R_matrix) if match found, else (None, None, None).
        """
        if not self.is_ready():
            return None, None, None

        self.last_update_time = self.timer.get_current_time()

        # 1. Define Search Area in the Global DEM
        row_est, col_est = self.dem_loader.lat_lon_to_pixel(est_lat, est_lon)

        # Define bounds (clamped to DEM size)
        r_start = max(0, row_est - search_radius_px)
        r_end = min(self.dem_loader.shape[0], row_est + search_radius_px)
        c_start = max(0, col_est - search_radius_px)
        c_end = min(self.dem_loader.shape[1], col_est + search_radius_px)

        # Extract the raw search area (un-normalized meters)
        search_area = self.dem_loader.data[r_start:r_end, c_start:c_end]

        # 2. Sliding Window Correlation
        best_mad = float('inf')
        best_rel_pixel = (0, 0)
        p_h, p_w = sensed_patch.shape  # (7, 7)

        # Iterate through every possible top-left corner in the search area
        for r in range(search_area.shape[0] - p_h):
            for c in range(search_area.shape[1] - p_w):
                # Extract candidate 7x7 sub-patch from the map
                candidate_sub = search_area[r:r + p_h, c:c + p_w]

                # Normalize candidate (z-score) to match the sensor's normalized patch
                norm_candidate = self.dem_loader._normalised_patch(candidate_sub)

                # Calculate Similarity: Mean Absolute Difference (MAD)
                # Lower is better (closer match)
                mad = np.mean(np.abs(norm_candidate - sensed_patch))

                if mad < best_mad:
                    best_mad = mad
                    best_rel_pixel = (r + p_h // 2, c + p_w // 2)

        # 3. Convert Best Local Match back to Geographic Coordinates
        final_row = r_start + best_rel_pixel[0]
        final_col = c_start + best_rel_pixel[1]

        measured_lat, measured_lon = self.dem_loader.pixel_to_lat_lon(final_row, final_col)

        # 4. Return Data + R-Matrix for Kalman Filter update
        return measured_lat, measured_lon, self.get_noise_covariance()

    def get_noise_covariance(self) -> np.ndarray:
        """Returns the R_TERCOM matrix for the Kalman Filter."""
        return np.diag([self.lateral_accuracy ** 2, self.lateral_accuracy ** 2, self.vertical_accuracy ** 2])