import numpy as np
from itertools import groupby
from scipy.interpolate import splprep, splev

class TrajectoryGenerator:
    def __init__(self, engine_backend, meter_per_z: float, meter_per_x_lookup: np.ndarray):
        self.engine = engine_backend
        self.meter_per_z = meter_per_z
        self.meter_per_x_lookup = meter_per_x_lookup

    def get_trajectory(self, raw_path: list[tuple[int, int]], smooth_factor: float, res_multi: int) -> list[tuple[int, int]]:
        """
        Smooth the path by B Spline (C2 continuity), which is aerodynamic and close to what missile could fly.
        Balancing continuity and local control.
        We ignore altitude (y will be handled by missile itself), we focus on x, z, making it still a 2d path.
        """

        # Check length
        if not raw_path or len(raw_path) < 3:
            return []

        clean_path = self._remove_duplication(raw_path)
        if len(clean_path) < 3: return []






    def _remove_duplication(self, path: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """
        Filter out any potential consecutive duplicated points that can cause error.
        We use groupby from itertools. Probably the fastest

        Args:
            - path: the list contains tuple (representing pixels)... yeah just path
        """
        pixels = np.array(path)
        diffs = np.diff(pixels)
        return [k for k, g in groupby(path)]


    def _compute_b_spline(self, clean_path: list[tuple[int, int]], smooth_factor: float, res_multi: int) -> list[tuple[int, int]]:
        """
        Smooth the path by b spline, using splev and splprep
        """
        try:
            # Numpy slicing
            row = clean_path[:, 0] # select everything but only keep the 0th index
            col = clean_path[:, 1] # same but 1st

            # Main smoothing part. Splprep for patter analysing, linspace for point generating, splev for drawing
            tck, u = splprep([row, col], s=smooth_factor * len(clean_path), k=3) # default k = 3, cubic
            u_new = np.linspace(0, 1, int(len(clean_path) * res_multi)) # draw original nodes * resolution upsampling to create smoother path
            new_row, new_col = splev(u_new, tck)

            return new_row.astype(np.float32), new_col.astype(np.float32)

        except Exception as e:
            print(f"Trajectory Gen Error: {e}")
            return None, None








