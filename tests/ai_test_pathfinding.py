import sys
import math
import random
import numpy as np
from pathlib import Path

# --- Setup Imports ---
# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

# Import your class
from src.guidance.pathfinding import Pathfinding

def pack_index(r, c, width):
    """Helper to simulate the internal index packing."""
    return r * width + c

def test_heuristic_logic(pf):
    print("\n=== TEST 1: Heuristic Logic (Packed Indices) ===")
    
    rows = pf.row
    cols = pf.col
    
    # Pick a random location safe from edges
    r, c = 1000, 1000
    start_idx = pack_index(r, c, cols)
    
    # 1. Identity Check (Distance to self)
    dist_self = pf._heuristic(start_idx, start_idx)
    if dist_self == 0.0:
        print("  ✅ Identity Passed (0.0m).")
    else:
        print(f"  ❌ Identity Failed: {dist_self}")

    # 2. Vertical Movement Check (Meters per Y)
    # Move 10 pixels down
    target_r = r + 10
    target_idx = pack_index(target_r, c, cols)
    
    dist_actual = pf._heuristic(start_idx, target_idx)
    dist_expected = 10 * pf.meter_per_y
    
    if abs(dist_actual - dist_expected) < 0.1:
        print(f"  ✅ Vertical Distance Accurate: {dist_actual:.2f}m")
    else:
        print(f"  ❌ Vertical Mismatch. Got {dist_actual}, Expected {dist_expected}")

    # 3. Horizontal Movement Check (Variable Width)
    # Move 10 pixels right
    target_c = c + 10
    target_idx = pack_index(r, target_c, cols)
    
    dist_horiz = pf._heuristic(start_idx, target_idx)
    
    # Verify it is consistent with the lookup table for this latitude
    width_at_lat = pf.meters_per_x_lookup[r]
    dist_expected_h = 10 * width_at_lat
    
    if abs(dist_horiz - dist_expected_h) < 0.1:
        print(f"  ✅ Horizontal Distance Accurate (Lat Corrected): {dist_horiz:.2f}m")
    else:
        print(f"  ❌ Horizontal Mismatch.")

def test_neighbors_boundaries(pf):
    print("\n=== TEST 2: Neighbors & Boundaries ===")
    
    rows = pf.row
    cols = pf.col

    # 1. Top-Left Corner (0,0) -> Should have 3 neighbors
    idx_corner = pack_index(0, 0, cols)
    neighbors = pf._get_neighbors(idx_corner) # Use the Packed method
    
    if len(neighbors) == 3:
        print(f"  ✅ Corner (0,0) has 3 neighbors.")
    else:
        print(f"  ❌ Corner (0,0) failed. Found {len(neighbors)} neighbors.")

    # 2. Bottom-Right Corner -> Should have 3 neighbors
    idx_end = pack_index(rows - 1, cols - 1, cols)
    neighbors = pf._get_neighbors(idx_end)
    
    if len(neighbors) == 3:
        print(f"  ✅ Corner ({rows-1},{cols-1}) has 3 neighbors.")
    else:
        print(f"  ❌ Bottom Corner failed.")

    # 3. Middle -> Should have 8 neighbors
    idx_mid = pack_index(500, 500, cols)
    neighbors = pf._get_neighbors(idx_mid)
    
    if len(neighbors) == 8:
        print(f"  ✅ Middle Node has 8 neighbors.")
    else:
        print(f"  ❌ Middle Node failed.")

def test_physics_calculation(pf):
    print("\n=== TEST 3: Physics & Terrain Sampling ===")
    
    rows, cols = pf.row, pf.col
    
    # Try to find a valid spot (not Ocean/NoData) to test physics
    attempts = 0
    found = False
    
    while attempts < 1000:
        r = random.randint(100, rows-100)
        c = random.randint(100, cols-100)
        
        curr_elev = pf.dem[r, c]
        neigh_elev = pf.dem[r, c+1] # Look East
        
        # Check if valid land
        if curr_elev > -100 and neigh_elev > -100:
            found = True
            
            # Pack indices
            curr_idx = pack_index(r, c, cols)
            neigh_idx = pack_index(r, c+1, cols)
            
            # Calculate Cost via Class
            calculated_cost = pf._get_movement_cost(curr_idx, neigh_idx)
            
            # Manual Check
            dist = pf._heuristic(curr_idx, neigh_idx)
            climb = neigh_elev - curr_elev
            
            print(f"  [Sample] Elev A: {curr_elev}m -> Elev B: {neigh_elev}m")
            print(f"  [Info]   Distance: {dist:.2f}m | Climb: {climb:.2f}m")
            
            if calculated_cost > dist:
                print(f"  ✅ Cost includes penalty/bonus (Total: {calculated_cost:.2f})")
            elif calculated_cost == float('inf'):
                print("  ⚠️ Hit infinite cost (unexpected for valid land).")
            else:
                print("  ✅ Cost valid.")
            break
            
        attempts += 1
        
    if not found:
        print("  ⚠️ Could not find valid land data for physics test.")

def test_full_integration(pf):
    print("\n=== TEST 4: Full Pathfinding Integration ===")
    
    # 1. Valid Short Path
    # Pick a known valid area (Siberia land)
    # 55N, 91E area is usually safe land
    try:
        start_r, start_c = pf.dem_loader.lat_lon_to_pixel(55.00, 92.0)
        end_r, end_c = pf.dem_loader.lat_lon_to_pixel(55.007, 92.012)
        
        print(f"  Requesting Path: ({start_r},{start_c}) -> ({end_r},{end_c})")
        
        path = pf.pathfinding((start_r, start_c), (end_r, end_c))
        
        if path and len(path) > 0:
            print(f"  ✅ Path Found! Length: {len(path)} steps.")
        else:
            print("  ❌ No path found (Unexpected for this location).")
            
    except Exception as e:
        print(f"  ❌ Crash during valid path test: {e}")

    # 2. Out of Bounds Input
    print("\n  [Testing Out of Bounds Input]")
    try:
        # Request path to (-500, -500)
        start_oob = (-50, -50)
        end_oob = (-10, -10)
        
        # This relies on your code handling the index packing math.
        # Ideally, it should catch it or return None, not crash with segfault.
        # Since Python allows negative indexing, it might wrap around map! 
        # But A* heuristics check for bounds in _get_neighbors.
        path = pf.pathfinding(start_oob, end_oob)
        if path is None:
            print("  ✅ Correctly returned None for OOB/Invalid Start.")
        else:
             print("  ⚠️ Warning: Path returned for OOB (Python negative indexing might have occurred).")
             
    except Exception as e:
        print(f"  ✅ Caught Expected Error: {e}")

    # 3. Same Start/End
    print("\n  [Testing Zero Length Path]")
    try:
        path = pf.pathfinding((100,100), (100,100))
        if path and len(path) == 1:
             print("  ✅ Start==End returns list with 1 node.")
        else:
             print(f"  ❌ Failed. Return: {path}")
    except Exception as e:
        print(f"  ❌ Crash: {e}")

if __name__ == "__main__":
    try:
        print("Initializing System...")
        pf = Pathfinding()
        
        # Run Tests
        test_heuristic_logic(pf)
        
        test_neighbors_boundaries(pf) 
        
        test_physics_calculation(pf)
        test_full_integration(pf)
        
        print("\n=== ALL TESTS COMPLETE ===")
        
    except FileNotFoundError:
        print("❌ CRITICAL: DEM File not found. Check path in Pathfinding class.")
    except Exception as e:
        print(f"❌ CRITICAL: Unexpected crash: {e}")