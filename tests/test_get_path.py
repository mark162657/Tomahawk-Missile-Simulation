"""
Simple test to run pathfinding with GPS coordinate, returning simplified path and time taken.


I handcoded this without AI... very tiring lol
"""

import sys
import time
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.guidance.pathfinding import Pathfinding

def test_pathfinding_gps(start: tuple[float, float], end: tuple[float, float]):

    pf = Pathfinding()
    # Accept both GPS and direct pixel (row, col) coordinate

    start_row, start_col = pf.dem_loader.lat_lon_to_pixel(start[0], start[1])
    end_row, end_col = pf.dem_loader.lat_lon_to_pixel(end[0], end[1])

    start_loc = (start_row, start_col)
    end_loc = (end_row, end_col)

    gps_dist = pf.get_surfcae_distance(start, end)

    print("\n============================================================")
    print("DATA RECEIVED...")
    print("============================================================")
    
    # Check if coordinates are within bounds
    if not (0 <= start_row < pf.row and 0 <= start_col < pf.col):
        print(f"\n! ERROR: Start coordinate out of bounds!")
        return None
    
    if not (0 <= end_row < pf.row and 0 <= end_col < pf.col):
        print(f"\n! ERROR: End coordinate out of bounds!")
        return None
    
    print(f"\nFinding path from... \n- Pixel: ({start_row}, {start_col}) -> ({end_row}, {end_col}) \n- GPS: {start} -> {end}\n")
    print(f"GPS distance: {gps_dist} meter ({gps_dist / 1000:.2f}km)")

    print("\n============================================================")
    print("START PATHFINDING")
    print("============================================================\n")
    
    start_time = time.time()
    pf.pathfinding(start_loc, end_loc)
    end_time = time.time()

    time_elapsed = end_time - start_time

    print(f"\nExecution Time: {time_elapsed:.3f} seconds")


if __name__ == "__main__":

    start_gps = (56.0, 94.0)
    end_gps = (55.20, 95.00)

    path = test_pathfinding_gps(start_gps, end_gps)

    print(path)
    

    
