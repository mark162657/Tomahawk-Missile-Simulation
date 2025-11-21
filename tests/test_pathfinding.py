"""
Full Test Suite for Pathfinding Module.
Tests: Heuristic Logic, Neighbors (Boundaries), and Movement Cost (Physics).
"""

import sys
from pathlib import Path
import numpy as np
import random
import math

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.guidance.pathfinding import Pathfinding

# --- CONFIGURATION ---
# Ensure this matches the file name inside your data/dem/ folder
DEM_FILENAME = "merged_dem_sib_N54_N59_E090_E100.tif" 

def test_heuristic_general(pathfinder):
    """
    Verifies general properties of the heuristic function using real map dimensions.
    """
    print("\n=== TEST 1: Heuristic Logic (Real Geometry) ===")
    
    rows = pathfinder.dem_loader.shape[0]
    cols = pathfinder.dem_loader.shape[1]
    
    print(f"  [Map Info] Loaded DEM: {rows} rows x {cols} cols")
    
    # Pick a random location (avoiding extreme edges)
    start_r = random.randint(100, rows - 200)
    start_c = random.randint(100, cols - 200)
    start = (start_r, start_c)
    
    # --- Check 1: Identity ---
    dist_self = pathfinder.heuristic(start, start)
    if dist_self == 0.0:
        print("  ✅ PASSED: Identity (Distance to self is 0).")
    else:
        print(f"  ❌ FAILED: Distance to self is {dist_self}")

    # --- Check 2: Pythagorean Consistency ---
    # Move 50px South, 50px East, and 50px Diagonal
    step = 50
    
    # Vertical (South)
    d_vert = pathfinder.heuristic(start, (start[0] + step, start[1]))
    # Horizontal (East)
    d_horiz = pathfinder.heuristic(start, (start[0], start[1] + step))
    # Diagonal (South-East)
    d_diag_actual = pathfinder.heuristic(start, (start[0] + step, start[1] + step))
    
    # Expected diagonal (a^2 + b^2 = c^2)
    d_diag_expected = math.sqrt(d_vert**2 + d_horiz**2)
    
    print(f"  [Math Check] Vert: {d_vert:.1f}m | Horiz: {d_horiz:.1f}m")
    print(f"  [Math Check] Diag Actual: {d_diag_actual:.1f}m | Expected: {d_diag_expected:.1f}m")
    
    # Allow 1.0m tolerance due to latitude distortion variance over the step distance
    if abs(d_diag_actual - d_diag_expected) < 1.0:
        print("  ✅ PASSED: Pythagorean consistency holds.")
    else:
        print("  ❌ FAILED: Diagonal calculation deviates.")


def test_neighbors_logic(pathfinder):
    """Verifies neighbor finding using the actual map boundaries."""
    print("\n=== TEST 2: Neighbor Finding (Real Boundaries) ===")
    
    rows = pathfinder.dem_loader.shape[0]
    cols = pathfinder.dem_loader.shape[1]
    
    # Case A: Random Center Node (Safe zone)
    rand_r = random.randint(50, rows - 50)
    rand_c = random.randint(50, cols - 50)
    center_node = (rand_r, rand_c)
    
    neighbors_c = pathfinder._get_neighbors(center_node)
    
    if len(neighbors_c) == 8:
        print(f"  ✅ PASSED: Center node {center_node} found 8 neighbors.")
    else:
        print(f"  ❌ FAILED: Center node found {len(neighbors_c)} neighbors.")

    # Case B: Top-Left Corner (0,0)
    corner_node = (0, 0)
    neighbors_corner = pathfinder._get_neighbors(corner_node)
    
    if len(neighbors_corner) == 3:
        print("  ✅ PASSED: Corner (0,0) found 3 neighbors.")
    else:
        print(f"  ❌ FAILED: Corner found {len(neighbors_corner)} neighbors.")


def test_physics_real_terrain(pathfinder):
    """
    Verifies movement costs by sampling REAL terrain data.
    It picks random points until it finds valid elevation data, then 
    manually calculates the expected cost to verify the function's logic.
    """
    print("\n=== TEST 3: Physics & Penalties (Real Terrain Sampling) ===")
    
    rows, cols = pathfinder.dem.shape
    nodata = pathfinder.dem_loader.nodata
    
    valid_sample_found = False
    attempts = 0
    
    # Loop until we find a valid spot (not water/nodata)
    while not valid_sample_found and attempts < 100:
        attempts += 1
        r = random.randint(0, rows - 2)
        c = random.randint(0, cols - 2)
        
        curr_elev = pathfinder.dem[r, c]
        
        # Ensure current node is valid
        if curr_elev == nodata or curr_elev < -100:
            continue
            
        # Pick a neighbor (East)
        target = (r, c + 1)
        neigh_elev = pathfinder.dem[target]
        
        # Ensure neighbor is valid
        if neigh_elev == nodata or neigh_elev < -100:
            continue
            
        valid_sample_found = True
        current = (r, c)
        
        print(f"  [Sample Found] Loc: {current} -> {target}")
        print(f"  [Elevation]    Curr: {curr_elev:.2f}m | Target: {neigh_elev:.2f}m")
        
        # 1. Run the actual function
        actual_cost = pathfinder._get_movement_cost(current, target)
        
        # 2. Manually calculate expected cost (Replicating logic for verification)
        dist_cost = pathfinder.heuristic(current, target)
        climb_height = neigh_elev - curr_elev
        
        expected_penalty = 0.0
        
        print(f"  [Geometry]     Dist: {dist_cost:.2f}m | Climb: {climb_height:.2f}m")

        # --- REPLICATE LOGIC ---
        if climb_height < 0:
            # Downhill
            expected_penalty = climb_height * 0.5
            print("  [Type]         DOWNHILL (Bonus)")
        else:
            # Uphill
            gradient = climb_height / (dist_cost + 1e-6)
            print(f"  [Type]         UPHILL | Gradient: {gradient:.4f}")
            
            
            
            if gradient <= 0.05:
                expected_penalty = climb_height * 5.0
                print("  [Penalty]      Level 1 (Gentle)")
            elif gradient <= 0.15:
                expected_penalty = climb_height * 20.0
                print("  [Penalty]      Level 2 (Moderate)")
            else:
                expected_penalty = climb_height * 100.0
                print("  [Penalty]      Level 3 (Steep)")

        expected_total = dist_cost + expected_penalty
        
        print(f"  Calculated Cost: {actual_cost:.4f}")
        print(f"  Expected Cost:   {expected_total:.4f}")
        
        if abs(actual_cost - expected_total) < 0.01:
            print("  ✅ PASSED: Physics calculation matches logic.")
        else:
            print("  ❌ FAILED: Calculation mismatch.")
            
    if not valid_sample_found:
        print("  ⚠️ WARNING: Could not find valid terrain data (NoData/Water) after 100 attempts.")


if __name__ == "__main__":
    # Initialize Logic (Loads the Real TIF)
    try:
        # Assuming Pathfinding loads the specific TIF internally or via arguments
        # If your Pathfinding class requires the path arg, pass it here.
        pathfinder = Pathfinding()
        
        # Run Suite
        test_heuristic_general(pathfinder)
        test_neighbors_logic(pathfinder)
        test_physics_real_terrain(pathfinder)
        
    except FileNotFoundError as e:
        print(f"\n❌ CRITICAL ERROR: {e}")
        print(f"Please check the path logic in src/guidance/pathfinding.py or the test file.")