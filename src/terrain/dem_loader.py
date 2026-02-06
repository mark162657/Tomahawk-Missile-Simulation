"""
Responsible
"""

import numpy as np
import matplotlib.pyplot as plt
import rasterio

from pathlib import Path
from rasterio.transform import rowcol, xy
from matplotlib.colors import LinearSegmentedColormap, LightSource

class DEMLoader:
    """Loads and queries elevation data from a single SRTM file."""

    def __init__(self, dem_path: Path) -> None:
        """
        Initialise DEM loader.

        For the data:
            read first band as NumPy array (rows, cols) = (height, width)
            so the index will be array[row, col]

        Args:
            dem_path: the path to the dem file
        """

        # Check existance
        self.path = Path(dem_path)
        if not self.path.exists():
            raise FileNotFoundError(f"DEM file not found: {self.path}. Check again.")

        # Load DEM data
        with rasterio.open(self.path) as src: # path will be provided by the user
            self.data = src.read(1)
            self.transform = src.transform # row, col <-> gps
            self.crs = src.crs
            self.bounds = src.bounds # see if coordinate is inside the bound
            self.shape = self.data.shape
            self.nodata = src.nodata # invalid data (pixel)


    def get_elevation(self, lat: float, lon: float) -> float | None:
        """
        Get elevation at GPS coordinates.

        Args:
            lat: Latitude (degrees)
            lon: Longitude (degrees)

        Returns:
            float: Elevation in meters, or None if out of bounds
        """
        try: 
            row, col = rowcol(self.transform, lon, lat)
        
            if 0 <= row < self.shape[0] and 0 <= col < self.shape[1]:
                elev = float(self.data[row, col])
                return elev if elev != self.nodata else None # return elevation if its not nodata
            return None
    
        except Exception as e:
            return None


    def get_elevation_patch(self, lat: float, lon: float, patch_size=7, normalized=True) -> np.ndarray[float]:
        """
        Getting a 7 x 7 patch around the centre coordinate (where the missile is located at).
        The patch will later be passed down for TERCOM navigation using Kalman filter.
        The patch will also be normalised. As we do not want a direct elevation comparison, rather a pattern and shape
        (simulates real-life).

        Args:
            - lat: current latitude
            - lon: current longitude

        Return:
            - the 2d np array consists of normalised patch data
        """
        # GPS coordinates -> pixels (row, col)
        pixel_row, pixel_col = rowcol(self.transform, lon, lat)
        half = patch_size // 2

        row_start = max(0, pixel_row - half)
        row_end = min(self.shape[0], pixel_row + half + 1)

        col_start = max(0, pixel_col - half)
        col_end = min(self.shape[1], pixel_col + half + 1)

        # still check for error nonetheless
        if row_start >= row_end or col_start >= col_end:
            return None

        # generate patch by slicing
        patch = self.data[row_start:row_end, col_start:col_end]

        # normalise the data (z-score normalisation):
        if normalized:
            patch = self._normalised_patch(patch)

        return patch

    def _normalised_patch(self, patch: np.ndarray) -> np.ndarray[float]:
        """
        Z-score normalization.
        """
        patch = patch.astype(float)  # Assign the result back to patch
        mean = np.mean(patch)
        std_dev = np.std(patch)

        # normalised and add Epsilon (prevent error of division-by-zero)
        normalised_patch = (patch - mean) / (std_dev + 1e-6)
        return normalised_patch

    def lat_lon_to_pixel(self, lat: float, lon: float):
        """Convert GPS to pixel coordinates."""
        return rowcol(self.transform, lon, lat)


    def pixel_to_lat_lon(self, row, col):
        """Convert pixel to GPS coordinates."""
        lon, lat = xy(self.transform, row, col)
        return lat, lon

# Quick test plotting
if __name__ == "__main__":
    script_dir = Path(__file__).resolve().parent
    # Set the root for the project
    project_root = script_dir.parents[1]
    # This guides where the tif file is located
    siberia_dem = "merged_dem_sib_N54_N59_E090_E100.tif"
    test_dem = "srtm_43_02.tif"
    dem_path = project_root / "data" / "dem" / siberia_dem

    dem = DEMLoader(dem_path)
    print(f"\n  DEM loaded: {dem.path.name}")
    print(f"  Shape: {dem.shape}")
    print(f"  Bounds: {dem.bounds}")

    # Use coordinates INSIDE the bounds of srtm_43_02.tif (30-35E, 50-55N)
    lat, lon = 55.5, 95.0
    elev = dem.get_elevation(lat, lon)

    if elev is not None:
        print(f"  Elevation at ({lat}, {lon}): {elev:.2f}m")
        
        # Test the patch extraction
        patch_size = 125
        patch = dem.get_elevation_patch(lat, lon, patch_size=patch_size)
        
        if patch is not None and patch.size > 0:
            print(f"\n  --- Patch Test ({patch_size}x{patch_size}) ---")
            print(f"  Patch shape: {patch.shape}")
            print(f"  Mean: {patch.mean():.4f} (expected ~0)")
            print(f"  Std Dev: {patch.std():.4f} (expected ~1)")
            print(f"  Full Normalized Patch Data:\n{patch}")
        else:
            print("  ⚠️ Patch is empty or out of bounds.")
    else:
        print(f"  ⚠️  Coordinate ({lat}, {lon}) outside tile bounds: {dem.bounds}")

        # ... rest of plotting code ...
