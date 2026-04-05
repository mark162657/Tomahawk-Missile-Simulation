import argparse
from pathlib import Path

import numpy as np

from src.terrain.dem_loader import DEMLoader


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEM_DIR = PROJECT_ROOT / "data" / "dem"
MAX_GPU_PIXELS = 4_000_000


def parse_waypoint(raw_value: str) -> tuple[float, float]:
    lat_str, lon_str = raw_value.split(",", maxsplit=1)
    return float(lat_str), float(lon_str)


def compute_window(
    dem: DEMLoader,
    mission_points: list[tuple[float, float]],
) -> tuple[int, int, int, int, int]:
    valid_points = [point for point in mission_points if dem.get_elevation(*point) is not None]

    if valid_points:
        rows = []
        cols = []
        for lat, lon in valid_points:
            row, col = dem.lat_lon_to_pixel(lat, lon)
            rows.append(row)
            cols.append(col)

        row_min = min(rows)
        row_max = max(rows)
        col_min = min(cols)
        col_max = max(cols)

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
    parser.add_argument("--waypoint", action="append", default=[])
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
    waypoints = [parse_waypoint(raw_value) for raw_value in args.waypoint]
    mission_points = [start, *waypoints, target]

    row_start, row_end, col_start, col_end, downsample = compute_window(dem, mission_points)
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

    valid_display_points = []
    for point in mission_points:
        if dem.get_elevation(*point) is None:
            continue
        row, col = dem.lat_lon_to_pixel(*point)
        valid_display_points.append([(col - col_start) / downsample, (row - row_start) / downsample])

    if len(valid_display_points) >= 2:
        path_data = np.array(valid_display_points, dtype=np.float32)
        area.add_line(data=path_data, colors="#13293d", thickness=2)

    if dem.get_elevation(*start) is not None:
        start_row, start_col = dem.lat_lon_to_pixel(*start)
        area.add_scatter(
            data=np.array([[(start_col - col_start) / downsample, (start_row - row_start) / downsample]], dtype=np.float32),
            colors="#00a86b",
            sizes=18,
            name="Start",
        )

    for waypoint in waypoints:
        if dem.get_elevation(*waypoint) is None:
            continue
        waypoint_row, waypoint_col = dem.lat_lon_to_pixel(*waypoint)
        area.add_scatter(
            data=np.array([[(waypoint_col - col_start) / downsample, (waypoint_row - row_start) / downsample]], dtype=np.float32),
            colors="#ffb000",
            sizes=15,
            name="Waypoint",
        )

    if dem.get_elevation(*target) is not None:
        target_row, target_col = dem.lat_lon_to_pixel(*target)
        area.add_scatter(
            data=np.array([[(target_col - col_start) / downsample, (target_row - row_start) / downsample]], dtype=np.float32),
            colors="#d1495b",
            sizes=18,
            name="Target",
        )

    area.name = f"GPU DEM Preview ({args.dem}, downsample={downsample}x)"
    figure.show(maintain_aspect=False)
    rendercanvas.auto.loop.run()


if __name__ == "__main__":
    main()
