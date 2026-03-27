import argparse
from pathlib import Path

import numpy as np

from src.terrain.dem_loader import DEMLoader


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEM_DIR = PROJECT_ROOT / "data" / "dem"
MAX_GPU_PIXELS = 4_000_000


def compute_window(
    dem: DEMLoader,
    start: tuple[float, float],
    target: tuple[float, float],
) -> tuple[int, int, int, int, int]:
    start_elev = dem.get_elevation(*start)
    target_elev = dem.get_elevation(*target)

    if start_elev is not None and target_elev is not None:
        start_row, start_col = dem.lat_lon_to_pixel(*start)
        target_row, target_col = dem.lat_lon_to_pixel(*target)
        row_min = min(start_row, target_row)
        row_max = max(start_row, target_row)
        col_min = min(start_col, target_col)
        col_max = max(start_col, target_col)

        row_padding = max(200, int((row_max - row_min) * 0.25))
        col_padding = max(200, int((col_max - col_min) * 0.25))

        row_start = max(0, row_min - row_padding)
        row_end = min(dem.shape[0], row_max + row_padding)
        col_start = max(0, col_min - col_padding)
        col_end = min(dem.shape[1], col_max + col_padding)
    else:
        row_start = 0
        row_end = dem.shape[0]
        col_start = 0
        col_end = dem.shape[1]

    height = max(1, row_end - row_start)
    width = max(1, col_end - col_start)
    downsample = 1
    while (height // downsample) * (width // downsample) > MAX_GPU_PIXELS:
        downsample += 1

    return row_start, row_end, col_start, col_end, downsample


def main() -> None:
    parser = argparse.ArgumentParser(description="GPU DEM preview for mission planning.")
    parser.add_argument("--dem", required=True)
    parser.add_argument("--start-lat", type=float, required=True)
    parser.add_argument("--start-lon", type=float, required=True)
    parser.add_argument("--target-lat", type=float, required=True)
    parser.add_argument("--target-lon", type=float, required=True)
    args = parser.parse_args()

    try:
        import fastplotlib as fpl
        import rendercanvas.auto
    except ImportError as exc:
        raise SystemExit(
            "fastplotlib/rendercanvas is not installed in this environment, so GPU preview is unavailable."
        ) from exc

    dem_path = DEM_DIR / args.dem
    dem = DEMLoader(dem_path)
    start = (args.start_lat, args.start_lon)
    target = (args.target_lat, args.target_lon)

    row_start, row_end, col_start, col_end, downsample = compute_window(dem, start, target)
    dem_patch = dem.data[row_start:row_end:downsample, col_start:col_end:downsample].astype(np.float32)

    if dem.nodata is not None:
        dem_patch[dem_patch == dem.nodata] = np.nan
    dem_patch[dem_patch <= -10000] = np.nan

    valid = dem_patch[~np.isnan(dem_patch)]
    if valid.size == 0:
        raise SystemExit("Selected DEM region contains no valid elevation values.")

    elev_min = float(np.percentile(valid, 2))
    elev_max = float(np.percentile(valid, 98))
    display_patch = np.nan_to_num(dem_patch, nan=elev_min)

    figure = fpl.Figure(size=(1400, 900))
    area = figure[0, 0]
    area.add_image(
        data=display_patch,
        cmap="terrain",
        vmin=elev_min,
        vmax=elev_max,
    )

    overlays = []
    start_valid = dem.get_elevation(*start) is not None
    target_valid = dem.get_elevation(*target) is not None

    if start_valid and target_valid:
        start_row, start_col = dem.lat_lon_to_pixel(*start)
        target_row, target_col = dem.lat_lon_to_pixel(*target)
        path_data = np.array(
            [
                [(start_col - col_start) / downsample, (start_row - row_start) / downsample],
                [(target_col - col_start) / downsample, (target_row - row_start) / downsample],
            ],
            dtype=np.float32,
        )
        overlays.append(area.add_line(data=path_data, colors="#13293d", thickness=2))
        overlays.append(area.add_scatter(data=path_data[:1], colors="#00a86b", sizes=15, name="Start"))
        overlays.append(area.add_scatter(data=path_data[1:], colors="#d1495b", sizes=15, name="Target"))

    area.name = f"GPU DEM Preview ({args.dem}, downsample={downsample}x)"
    figure.show(maintain_aspect=False)
    rendercanvas.auto.loop.run()


if __name__ == "__main__":
    main()
