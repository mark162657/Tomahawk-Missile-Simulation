"""
Visualize pathfinding results on a downsampled DEM to compare path quality across different heuristic weights.
"""

import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.guidance.pathfinding import Pathfinding
import rasterio
from matplotlib.colors import LinearSegmentedColormap

def visualize_path_comparison(start_gps: tuple[float, float], end_gps: tuple[float, float], 
                               weights: list[float] = [1.0, 1.1, 1.2, 1.5, 2.0]):
    """
    Run pathfinding with multiple heuristic weights and visualize all paths on the same map.
    
    Args:
        start_gps: (lat, lon) start coordinate
        end_gps: (lat, lon) end coordinate
        weights: List of heuristic weights to test
    """
    
    print("\n" + "="*60)
    print("PATH QUALITY COMPARISON")
    print("="*60)
    
    # Initialize pathfinding
    pf = Pathfinding()
    
    # Convert GPS to pixel coordinates
    start_row, start_col = pf.dem_loader.lat_lon_to_pixel(start_gps[0], start_gps[1])
    end_row, end_col = pf.dem_loader.lat_lon_to_pixel(end_gps[0], end_gps[1])
    
    start_pixel = (start_row, start_col)
    end_pixel = (end_row, end_col)
    
    print(f"\nStart GPS: {start_gps} → Pixel: {start_pixel}")
    print(f"End GPS:   {end_gps} → Pixel: {end_pixel}")
    
    # Compute paths for each weight
    paths_data = []
    
    for weight in weights:
        print(f"\n--- Testing Weight: {weight} ---")
        start_time = time.time()
        
        path = pf.pathfinding(start_pixel, end_pixel, heuristic_weight=weight)
        
        elapsed = time.time() - start_time
        
        if path:
            path_length = len(path)
            
            # Calculate total distance
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
            
            print(f"✅ Path found: {path_length} nodes, {total_dist/1000:.2f}km, {total_climb:.0f}m climb")
            print(f"   Time: {elapsed:.3f}s")
        else:
            print(f"❌ No path found")
    
    # Visualization
    print("\n" + "="*60)
    print("GENERATING VISUALIZATION...")
    print("="*60)
    
    # Calculate bounding box for all paths with padding
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
    
    print(f"Zoom area: rows [{min_row}:{max_row}], cols [{min_col}:{max_col}]")
    
    # Extract zoomed DEM region with downsampling
    downsample = 2
    dem_region = pf.dem[min_row:max_row:downsample, min_col:max_col:downsample].astype(float)
    dem_region[dem_region <= -100] = np.nan  # Remove nodata
    
    # Create color map
    colors = ['#2d5016', '#5a8c2b', '#8fb359', '#d4c77e', '#a67c52', '#ffffff']
    cmap_terrain = LinearSegmentedColormap.from_list('terrain', colors, N=100)
    
    # Setup plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))
    
    # --- LEFT PLOT: All paths overlaid (ZOOMED) ---
    ax1.imshow(dem_region, cmap=cmap_terrain, aspect='auto', interpolation='bilinear',
               extent=[min_col//downsample, max_col//downsample, max_row//downsample, min_row//downsample])
    ax1.set_title('Path Comparison: All Heuristic Weights (Zoomed)', fontsize=14, fontweight='bold')
    ax1.set_xlabel('Column (pixels)', fontsize=11)
    ax1.set_ylabel('Row (pixels)', fontsize=11)
    
    # Plot paths with different colors
    path_colors = plt.cm.rainbow(np.linspace(0, 1, len(paths_data)))
    
    for i, data in enumerate(paths_data):
        path = data['path']
        weight = data['weight']
        
        # Downsample path coordinates
        path_rows = [p[0] // downsample for p in path]
        path_cols = [p[1] // downsample for p in path]
        
        ax1.plot(path_cols, path_rows, 
                 color=path_colors[i], 
                 linewidth=3.0, 
                 alpha=0.85,
                 label=f"w={weight} ({data['length']} nodes, {data['distance']/1000:.1f}km)")
    
    # Mark start and end
    ax1.scatter(start_col // downsample, start_row // downsample, 
                c='lime', s=200, marker='o', edgecolors='black', linewidths=2.5, 
                label='Start', zorder=10)
    ax1.scatter(end_col // downsample, end_row // downsample, 
                c='red', s=200, marker='X', edgecolors='black', linewidths=2.5, 
                label='End', zorder=10)
    
    ax1.legend(loc='best', fontsize=9, framealpha=0.95)
    ax1.grid(True, alpha=0.3, linewidth=0.5)
    
    # --- RIGHT PLOT: Performance metrics ---
    ax2.axis('off')
    ax2.set_title('Performance Metrics', fontsize=14, fontweight='bold')
    
    # Create table
    table_data = []
    headers = ['Weight', 'Nodes', 'Distance (km)', 'Climb (m)', 'Time (s)']
    
    for data in paths_data:
        table_data.append([
            f"{data['weight']:.1f}",
            f"{data['length']}",
            f"{data['distance']/1000:.2f}",
            f"{data['climb']:.0f}",
            f"{data['time']:.3f}"
        ])
    
    table = ax2.table(cellText=table_data, colLabels=headers,
                      cellLoc='center', loc='center',
                      colWidths=[0.12, 0.12, 0.18, 0.15, 0.15])
    
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 2.5)
    
    # Style header
    for i in range(len(headers)):
        table[(0, i)].set_facecolor('#4CAF50')
        table[(0, i)].set_text_props(weight='bold', color='white')
    
    # Alternate row colors
    for i in range(1, len(table_data) + 1):
        for j in range(len(headers)):
            if i % 2 == 0:
                table[(i, j)].set_facecolor('#f0f0f0')
    
    plt.tight_layout()
    
    # Save figure
    output_path = Path(__file__).parent.parent / 'data' / 'results' / f'path_comparison_{int(time.time())}.png'
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n💾 Visualization saved: {output_path}")
    
    plt.show()
    
    # Print summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    best_time = min(paths_data, key=lambda x: x['time'])
    best_distance = min(paths_data, key=lambda x: x['distance'])
    best_climb = min(paths_data, key=lambda x: x['climb'])
    
    print(f"⚡ Fastest: Weight {best_time['weight']} ({best_time['time']:.3f}s)")
    print(f"📏 Shortest: Weight {best_distance['weight']} ({best_distance['distance']/1000:.2f}km)")
    print(f"⛰️  Least Climb: Weight {best_climb['weight']} ({best_climb['climb']:.0f}m)")


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
    
    # Run visualization with multiple weights
    visualize_path_comparison(start_gps, end_gps, weights=[1.0, 1.5, 2.0])