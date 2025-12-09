import numpy as np
from scipy import interpolate

class TrajectoryGenerator:
    def __init__(self, engine_backend, meter_per_z: float, meter_per_x_lookup: np.ndarray):
        self.engine = engine_backend
        self.meter_per_z = meter_per_z
        self.meter_per_x_lookup = meter_per_x_lookup

    def get_trajectory(self, raw_path: list[tuple[int, int]]):
        """
        Smooth the path by B Spline (C2 continuity), which is aerodynamic and close to what missile could fly.
        Balancing continuity and local control.
        We ignore altitude (y will be handled by missile itself), we focus on x, z, making it still a 2d path.
        """

        # Check length
        if not raw_path or len(raw_path) < 3:
            return []





    def _remove_duplication(self):
        """
        Filter out any potential consecutive duplicated points that can cause error.
        """
