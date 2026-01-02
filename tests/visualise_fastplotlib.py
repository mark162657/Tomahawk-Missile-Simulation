"""
2D Fastplotlib visualization of pathfinding results on DEM terrain.
Shows multiple paths with different heuristic weights overlaid on terrain heatmap.
Automatically uses full resolution when zoomed area is small enough.
"""

import sys
import time
import numpy as np
from pathlib import Path
import fastplotlib as fpl

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent))

from ..src.guidance.pathfinding_backend import Pathfinding


def visualize_path_fastplotlib(start_gps: tuple[float, float], end_gps: tuple[float, float], 
                                weights: list[float] = [1.5]):
    """
    Run pathfinding with multiple heuristic weights and visualize all paths in 2D.
    
    Args:
        start_gps: (lat, lon) start coordinate
        end_gps: (lat, lon) end coordinate
        weights: List of heuristic weights to test
    """
    
    print("\n" + "="*60)
    print("2D PATH VISUALIZATION (FASTPLOTLIB)")
    print("="*60)
    
    # Initialize pathfinding
    pf = Pathfinding()
    
    # Convert GPS to pixel coordinates
    start_row, start_col = pf.dem_loader.lat_lon_to_pixel(start_gps[0], start_gps[1])
    end_row, end_col = pf.dem_loader.lat_lon_to_pixel(end_gps[0], end_gps[1])
    
    start_pixel = (start_row, start_col)
    end_pixel = (end_row, end_col)
    
    print(f"\nStart GPS: {start_gps} -> Pixel: {start_pixel}")
    print(f"End GPS:   {end_gps} -> Pixel: {end_pixel}")
    
    # Compute paths for each weight
    paths_data = []
    
    for weight in weights:
        print(f"\n--- Testing Weight: {weight} ---")
        start_time = time.time()
        
        path = pf.pathfinding(start_pixel, end_pixel, heuristic_weight=weight)
        
        elapsed = time.time() - start_time
        
        if path:
            path_length = len(path)
            
            # Calculate total distance and climb
            total_dist = 0.0
            total_climb = 0.0
            for i in range(len(path) - 1):
                curr_idx = pf.pixel_to_idx(path[i][0], path[i][1])
                next_idx = pf.pixel_to_idx(path[i+1][0], path[i+1][1])
                total_dist += pf._heuristic(curr_idx, next_idx)
                
                # Calculate elevation change
                curr_elev = pf.dem[path[i]]
                next_elev = pf.dem[path[i+1]]
                climb = max(0, next_elev - curr_elev)
                total_climb += climb
            
            paths_data.append({
                'weight': weight,
                'path': path,
                'length': path_length,
                'distance': total_dist,
                'climb': total_climb,
                'time': elapsed
            })
            
            print(f"[OK] Path found: {path_length} nodes, {total_dist/1000:.2f}km, {total_climb:.0f}m climb")
            print(f"   Time: {elapsed:.3f}s")
        else:
            print(f"[ERROR] No path found")
    
    if not paths_data:
        print("\n[ERROR] No paths found. Exiting.")
        return
    
    # Calculate bounding box for all paths with padding
    print("\n" + "="*60)
    print("GENERATING 2D VISUALIZATION...")
    print("="*60)
    
    all_rows = []
    all_cols = []
    for data in paths_data:
        for point in data['path']:
            all_rows.append(point[0])
            all_cols.append(point[1])
    
    min_row, max_row = min(all_rows), max(all_rows)
    min_col, max_col = min(all_cols), max(all_cols)
    
    # Add padding (20% of path dimensions)
    row_padding = int((max_row - min_row) * 0.2) + 100
    col_padding = int((max_col - min_col) * 0.2) + 100
    
    min_row = max(0, min_row - row_padding)
    max_row = min(pf.row - 1, max_row + row_padding)
    min_col = max(0, min_col - col_padding)
    max_col = min(pf.col - 1, max_col + col_padding)
    
    # Capture tight bounds for color scaling (optimizes contrast for the path area)
    tight_min_row, tight_max_row = min_row, max_row
    tight_min_col, tight_max_col = min_col, max_col
    
    # Force square aspect ratio to prevent stretching/strip effect
    # This creates the "100km x 100km" graph context for long paths
    height = max_row - min_row
    width = max_col - min_col
    target_dim = max(height, width)
    
    if height < target_dim:
        diff = target_dim - height
        min_row = max(0, min_row - diff // 2)
        max_row = min(pf.row - 1, max_row + diff // 2)
        
    if width < target_dim:
        diff = target_dim - width
        min_col = max(0, min_col - diff // 2)
        max_col = min(pf.col - 1, max_col + diff // 2)
    
    print(f"Zoom area: rows [{min_row}:{max_row}], cols [{min_col}:{max_col}]")
    
    # Auto-adjust downsample to maximize detail while staying within pixel limit
    max_pixels = 20_000_000  # 20 million pixels max for smooth performance
    zoom_rows = max_row - min_row
    zoom_cols = max_col - min_col
    original_pixels = zoom_rows * zoom_cols
    
    # Start with downsample=1 (full res) and increase if needed
    optimal_downsample = 1
    while True:
        downsampled_pixels = (zoom_rows // optimal_downsample) * (zoom_cols // optimal_downsample)
        if downsampled_pixels <= max_pixels:
            break
        optimal_downsample += 1
    
    downsample = optimal_downsample
    
    if downsample == 1:
        print(f"Using FULL RESOLUTION ({original_pixels:,} pixels)")
    else:
        print(f"Auto-adjusted downsample to {downsample} ({original_pixels:,} -> {downsampled_pixels:,} pixels)")
    
    # Extract zoomed DEM region with downsampling
    dem_region = pf.dem[min_row:max_row:downsample, min_col:max_col:downsample].astype(float)
    dem_region[dem_region <= -100] = np.nan  # Remove nodata
    
    # Calculate stats from tight region to optimize contrast for the path
    # We extract the tight region using the same downsample factor
    stats_region = pf.dem[tight_min_row:tight_max_row:downsample, tight_min_col:tight_max_col:downsample].astype(float)
    stats_region[stats_region <= -100] = np.nan
    valid_elev = stats_region[~np.isnan(stats_region)]
    
    if len(valid_elev) > 0:
        elev_min = np.percentile(valid_elev, 2)
        elev_max = np.percentile(valid_elev, 98)
    else:
        # Fallback if tight region is empty (unlikely)
        valid_elev_full = dem_region[~np.isnan(dem_region)]
        elev_min = np.percentile(valid_elev_full, 2)
        elev_max = np.percentile(valid_elev_full, 98)
    
    print(f"Elevation range (optimized for path): {elev_min:.0f}m - {elev_max:.0f}m")
    print(f"Terrain shape: {dem_region.shape} ({dem_region.size:,} pixels)")
    
    # Replace NaN for visualization
    dem_display = np.nan_to_num(dem_region, nan=elev_min)
    
    # Create figure
    figure = fpl.Figure(size=(1400, 900))
    
    # Add terrain heatmap
    heatmap = figure[0, 0].add_image(
        data=dem_display,
        cmap="gist_ncar",  # High contrast colormap
        vmin=elev_min,
        vmax=elev_max,
    )
    
    # Set title
    figure[0, 0].name = f"Path Comparison (Elevation: {elev_min:.0f}m - {elev_max:.0f}m)"
    
    # Add paths with different colors
    path_colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan']
    
    for i, data in enumerate(paths_data):
        path = data['path']
        weight = data['weight']
        
        # Convert path to downsampled coordinates
        path_2d = []
        for row, col in path:
            # Convert to local downsampled coordinates
            ds_row = (row - min_row) // downsample
            ds_col = (col - min_col) // downsample
            
            # Check bounds
            if 0 <= ds_row < dem_region.shape[0] and 0 <= ds_col < dem_region.shape[1]:
                path_2d.append([ds_col, ds_row])  # Note: (x, y) = (col, row)
        
        path_2d = np.array(path_2d, dtype=np.float32)
        
        # Add line to figure
        if len(path_2d) > 0:
            path_line = figure[0, 0].add_line(
                data=path_2d,
                thickness=3,
                colors=path_colors[i % len(path_colors)],
                name=f"w={weight} ({data['length']} nodes, {data['distance']/1000:.1f}km)"
            )
    
    # Add start marker
    start_ds_row = (start_row - min_row) // downsample
    start_ds_col = (start_col - min_col) // downsample
    if 0 <= start_ds_row < dem_region.shape[0] and 0 <= start_ds_col < dem_region.shape[1]:
        start_marker = figure[0, 0].add_scatter(
            data=np.array([[start_ds_col, start_ds_row]], dtype=np.float32),
            sizes=20,
            colors="lime",
            name="Start"
        )
    
    # Add end marker
    end_ds_row = (end_row - min_row) // downsample
    end_ds_col = (end_col - min_col) // downsample
    if 0 <= end_ds_row < dem_region.shape[0] and 0 <= end_ds_col < dem_region.shape[1]:
        end_marker = figure[0, 0].add_scatter(
            data=np.array([[end_ds_col, end_ds_row]], dtype=np.float32),
            sizes=20,
            colors="red",
            name="End"
        )
    
    # Print summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    
    for data in paths_data:
        print(f"\nWeight {data['weight']}:")
        print(f"  Nodes: {data['length']}")
        print(f"  Distance: {data['distance']/1000:.2f} km")
        print(f"  Climb: {data['climb']:.0f} m")
        print(f"  Time: {data['time']:.3f} s")
    
    best_time = min(paths_data, key=lambda x: x['time'])
    best_distance = min(paths_data, key=lambda x: x['distance'])
    best_climb = min(paths_data, key=lambda x: x['climb'])
    
    print(f"\n[FASTEST] Weight {best_time['weight']} ({best_time['time']:.3f}s)")
    print(f"[SHORTEST] Weight {best_distance['weight']} ({best_distance['distance']/1000:.2f}km)")
    print(f"[LEAST CLIMB] Weight {best_climb['weight']} ({best_climb['climb']:.0f}m)")
    
    print("\n>> Starting viewer...")
    print("Press Ctrl+C or close the window to exit...")
    figure.show(maintain_aspect=True)  # Maintain aspect ratio to avoid stretching
    
    # Keep window open
    import rendercanvas.auto
    rendercanvas.auto.loop.run()


if __name__ == "__main__":
    # Test configuration
    print("\nSelect test scenario:")
    print("1. Short path (~10km)")
    print("2. Medium path (~50km)")
    print("3. Long path (~100km)")
    print("4. Custom coordinates")
    
    choice = input("\nEnter choice (1-4): ").strip()
    
    # Base coordinate (Siberia safe land)
    base_lat, base_lon = 56.0, 95.0
    
    # Degrees per km at latitude 56°
    deg_per_km_lon = 1 / (111.32 * 0.559193)
    deg_per_km_lat = 1 / 111.32
    
    if choice == "1":
        # 10km east
        start_gps = (base_lat, base_lon)
        end_gps = (base_lat, base_lon + 10 * deg_per_km_lon)
        
    elif choice == "2":
        # 50km northeast
        start_gps = (base_lat, base_lon)
        end_gps = (base_lat + 25 * deg_per_km_lat, base_lon + 43.3 * deg_per_km_lon)
        
    elif choice == "3":
        # 100km east
        start_gps = (base_lat, base_lon)
        end_gps = (base_lat, base_lon + 100 * deg_per_km_lon)
        
    elif choice == "4":
        print("\nEnter custom coordinates:")
        start_lat = float(input("Start Latitude: "))
        start_lon = float(input("Start Longitude: "))
        end_lat = float(input("End Latitude: "))
        end_lon = float(input("End Longitude: "))
        
        start_gps = (start_lat, start_lon)
        end_gps = (end_lat, end_lon)
    else:
        print("Invalid choice. Using default 10km test.")
        start_gps = (base_lat, base_lon)
        end_gps = (base_lat, base_lon + 10 * deg_per_km_lon)
    
    # Run 2D visualization with multiple weights
    visualize_path_fastplotlib(start_gps, end_gps, weights=[2])
