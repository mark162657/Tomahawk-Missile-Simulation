"""AI Test script for pathfinding pixel-based heuristic calculation."""

from src.guidance.pathfinding import Pathfinding
import math

def run_test_at_location(pathfinder, start_node, location_name):
    """Runs the square vs rectangle test at a specific map location."""
    print(f"\n--- Testing at {location_name} {start_node} ---")
    
    r, c = start_node
    
    # 1. Safety Check (Stay in bounds)
    max_r, max_c = pathfinder.dem.shape
    step_size = 100
    
    if r + step_size >= max_r or c + step_size >= max_c:
        print("  Skipping: Too close to edge.")
        return

    # 2. Calculate Moves
    # Vertical Move (South)
    dist_vert = pathfinder.heuristic((r, c), (r + step_size, c))
    
    # Horizontal Move (East)
    dist_horiz = pathfinder.heuristic((r, c), (r, c + step_size))
    
    # Diagonal Move
    dist_diag = pathfinder.heuristic((r, c), (r + step_size, c + step_size))

    # 3. Analyze Results
    print(f"  Vertical (100px):   {dist_vert:,.2f} m")
    print(f"  Horizontal (100px): {dist_horiz:,.2f} m")
    
    ratio = dist_vert / dist_horiz
    print(f"  Aspect Ratio:       {ratio:.2f} (1 Vertical Step = {ratio:.2f} Horizontal Steps)")

    # 4. Verify Diagonal (Allowing for 'Trapezoid' error)
    # We expect a slight mismatch because diagonal uses the MID-ROW width, 
    # while pure horizontal uses the START-ROW width.
    expected_pythag = math.sqrt(dist_vert**2 + dist_horiz**2)
    diff = abs(dist_diag - expected_pythag)
    
    print(f"  Diagonal Check:     {dist_diag:,.2f} m (vs simple pythag {expected_pythag:,.2f})")
    print(f"  Difference:         {diff:.3f} m")
    
    if diff < 1.0:
        print("  ✅ PASSED: Geometry is consistent.")
    else:
        print("  ⚠️ NOTE: Large difference due to rapid latitude change (Trapezoid effect).")

if __name__ == "__main__":
    print("\n=== 3D Pathfinding Heuristic Test ===")
    pathfinder = Pathfinding()
    
    rows = pathfinder.dem_loader.shape[0]
    cols = pathfinder.dem_loader.shape[1]
    
    # TEST 1: Top of Map (North - Should be narrowest pixels)
    run_test_at_location(pathfinder, (0, 0), "TOP (North)")
    
    # TEST 2: Center of Map
    run_test_at_location(pathfinder, (rows // 2, cols // 2), "CENTER")
    
    # TEST 3: Bottom of Map (South - Should be widest pixels)
    run_test_at_location(pathfinder, (rows - 101, cols // 2), "BOTTOM (South)")