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


    def get_elevation_patch(self, lat: float, lon: float, patch_size=7) -> np.ndarray[float]:
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
        if row_start < 0 or row_end > self.shape[0] or col_start < 0 or col_end > self.shape[1]:
            return None  # Out of bounds!

        # generate patch by slicing: a numpy 2d array [[...], [...]]
        patch = self.data[row_start:row_end, col_start:col_end] # as of dem, slicing adds the elevation automatically

        # normalise the data (z-score normalisation):
        patch = self._normalised_patch(patch)

        return patch

    def _normalised_patch(self, patch: np.ndarray) -> np.ndarray[float]:
        """

        """
        patch.astype(float)
        mean = np.mean(patch)
        std_dev = np.std(patch)

        # normalised and add Epsilon (prevent error of division-by-zero, especially with a perfectly flat patch)
        normalised_patch = (patch - mean) / (std_dev + 1e-6) # 1e-6, too small to affect data, but prevent division error
        return normalised_patch


    def lat_lon_to_pixel(self, lat: float, lon: float):
        """Convert GPS to pixel coordinates."""
        return rowcol(self.transform, lon, lat)


    def pixel_to_lat_lon(self, row, col):
        """Convert pixel to GPS coordinates."""
        lon, lat = xy(self.transform, row, col)
        return lat, lon

# Quick test
if __name__ == "__main__":
    script_dir = Path(__file__).resolve().parent
    # Set the root for the project
    project_root = script_dir.parents[1]
    # This guides where the tif file is located, for testing
    siberia_dem = "merged_dem_sib_N54_N59_E090_E100.tif"
    test_dem = "srtm_43_02.tif"
    dem_path = project_root / "data" / "dem" / test_dem

    dem = DEMLoader(dem_path)
    print(f"\n  DEM loaded: {dem.path.name}")
    print(f"  Shape: {dem.shape}")
    print(f"  Bounds: {dem.bounds}")

    # Test the lat/lon to elevation query, should be in range
    lat, lon = 56.0, 95.0
    elev = dem.get_elevation(lat, lon)

    elev1 = dem.get_elevation(55.5, 92.3)
    elev2 = dem.get_elevation(58.2, 97.7)

    print(f"  Elevation at 55.5, 92.3 is {elev1}")
    print(f"  Elevation at 58.2, 97.7 is {elev2}")
    
    if elev:
        print(f"  Elevation at ({lat}, {lon}): {elev:.2f}m")
    else:
        print(f"  ⚠️  Coordinate ({lat}, {lon}) outside tile bounds")

    # Load DEM data for plotting and handle NoData values
    with rasterio.open(dem_path) as src:
        dem_data = src.read(1).astype(float)  # Convert to float for NaN handling
        nodata = src.nodata
        bounds = src.bounds

        # Replace NoData values with NaN to prevent matplotlib errors
        if nodata is not None:
            dem_data[dem_data == nodata] = np.nan

    # Handle downsampling
    downsample_size = 2
    dem_downsampled = dem_data[::downsample_size, ::downsample_size]

    # Print console
    print(f"\n  Visualization Info:\n")
    print(f"  Original shape: {dem_data.shape} | Total pixels: {dem_data.size:,}") # show shape and total pixels
    print(f"  Downsampled shape: {dem_downsampled.shape} | Total pixels: {dem_downsampled.size:,}")
    print(f"  Reduction by: {dem_data.size / dem_downsampled.size:.0f}x")

    # green -> yellow -> brown -> white
    colors = ['#2d5016', '#5a8c2b', '#8fb359', '#d4c77e', '#a67c52', '#ffffff']
    n_bins = 100
    cmap_custom = LinearSegmentedColormap.from_list('terrain_land', colors, N=n_bins)

    vmin = np.nanpercentile(dem_downsampled, 2)   # 2nd percentile
    vmax = np.nanpercentile(dem_downsampled, 98)  # 98th percentile

    ls = LightSource(azdeg=315, altdeg=45)
    hillshade = ls.hillshade(dem_downsampled, vert_exag=2, dx=1, dy=1)


    # Plot DEM
    plt.figure(figsize=(14, 10))
    plt.imshow(hillshade, cmap='gray', extent=(bounds.left, bounds.right, bounds.bottom, bounds.top), alpha=0.3)
    plt.imshow(dem_downsampled, cmap=cmap_custom, aspect='auto', extent=(bounds.left, bounds.right, bounds.bottom, bounds.top), interpolation='bilinear', alpha=0.7, vmin=vmin, vmax=vmax)
    plt.colorbar(label='Elevation (meters)', fraction=0.046, pad=0.04)
    plt.title(f'Downsampled 1:{downsample_size} DEM', fontsize=14, fontweight='bold')
    plt.xlabel('Longitude (degrees)', fontsize=12)
    plt.ylabel('Latitude (degrees)', fontsize=12)
    plt.gca().invert_yaxis()  # having north on top (traditional map layout)
    plt.tight_layout()
    print('\nPlot generating')
    plt.show()


