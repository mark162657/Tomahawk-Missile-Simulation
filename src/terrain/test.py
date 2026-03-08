import numpy as np
import fastplotlib as fpl
from pathlib import Path
from src.terrain.dem_loader import DEMLoader


def visualize_dem_fast(dem_path: Path):
    loader = DEMLoader(dem_path)
    plot_data = loader.data.astype(np.float32)

    # Handle NoData for visual clarity
    valid_mask = plot_data != loader.nodata
    min_elev = np.nanmin(plot_data[valid_mask])
    plot_data[~valid_mask] = min_elev

    figure = fpl.Figure(size=(1200, 800))
    figure[0, 0].title = f"DEM Viewer: {dem_path.name}"

    # Add image and name it
    dem_graphic = figure[0, 0].add_image(plot_data, cmap="terrain", name="terrain_map")

    info_text = figure[0, 0].add_text("Hover to inspect terrain...", offset=(10, 10, 0))

    def update_info(event_data):
        pos = event_data.world_pos
        col, row = int(pos[0]), int(pos[1])

        if 0 <= row < loader.shape[0] and 0 <= col < loader.shape[1]:
            lat, lon = loader.pixel_to_lat_lon(row, col)
            elevation = loader.data[row, col]
            status = f"Lat: {lat:.4f}, Lon: {lon:.4f} | Elev: {elevation}m"
            info_text.text = status

    # Recommended: Attach event handler to the graphic itself
    dem_graphic.add_event_handler(update_info, "pointer_move")

    figure.show()

    # CRITICAL: Start the event loop to keep the window open
    fpl.loop.run()


if __name__ == "__main__":
    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parents[1]
    iran_dem = project_root / "data" / "dem" / "merged_dem_iran_N31_N41_E44_E54.tif"

    if iran_dem.exists():
        visualize_dem_fast(iran_dem)
    else:
        print(f"File not found: {iran_dem}")