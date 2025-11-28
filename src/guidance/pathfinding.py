import heapq
from multiprocessing import heap
from ..terrain.dem_loader import DEMLoader
import math
from pathlib import Path
import numpy as np


class Pathfinding:
    """
    Using A*pathfinding algorithm to find the most ideal path that considers the horizontal and vertical movement.
    Before acheiving HPA or RHA pathfinding which is more efficient, or even downsampling the map, the program will
        be heavily focused on saving RAM usage and incease efficiency.
    
    """
    def __init__(self):
        tif_path = Path(__file__).parent.parent.parent / 'data' / 'dem' / 'merged_dem_sib_N54_N59_E090_E100.tif'
        dem = DEMLoader(tif_path)
        self.dem_loader = dem
        self.dem = dem.data

        # Vertical (north / south) setup - y
        self.meter_per_y = abs(self.dem_loader.transform[4] * 111320) # 111320 is meter per degree (lat)

        # Initiating the lookup table and column value for future use
        self.row = self.dem_loader.shape[0] # Shape: (rows, columns)
        self.col = self.dem_loader.shape[1]
        
        # Creating a numpy array
        row_indicies = np.arange(self.row)

        # Get the starting latitude (top left corner according to rasterio)
        start_lat = self.dem_loader.transform[5]

        # Get the step size (how much lat changes per pixel)
        pixel_height = self.dem_loader.transform[4]

        # Calculate the latitude for every single row at once
        self.latitudes = start_lat + (row_indicies * pixel_height)

        # Horizontal (east / west) setup - x
        base_width_meters = abs(self.dem_loader.transform[0] * 111320)
        self.meters_per_x_lookup = base_width_meters * np.cos(np.radians(self.latitudes))

        print("Lookup table generated")


    def _heuristic(self, loc1_idx: int, loc2_idx: int) -> float:
        """
        Calculate the heuristic distance between two pixel (locations) using Euclidean distance.

        Corrected for longitudinal distortion by scaling horizontal distance based on latitude and Earth curvature.
        This could minimise around 15% of distance error. Compared to non-corrected version.
        Stored and reference using lookup table to improve performance.

        Compared th previous function: get_surface_distance, this drastically lower the calculation and time complexity.
        Require the program to not calculating the distance again and again each time it need to get the H-score.

        Args:
            - loc1 and loc2: tuple that stores the row, col informtion

        Return:
            - Euclidean distacnce between two points (also as H-score)

        """
        
        # Location node
        row1, col1 = divmod(loc1_idx, self.col)
        row2, col2 = divmod(loc2_idx, self.col)

        # Get vertical distance (north/south direction)
        dist_y = (row2 - row1) * self.meter_per_y

        # Midrow between two locations, used for average latitude between two points, as horizontal width change with latitude, more accurate distance estimate and saves resource (not getting width for every position)
        midrow = int((row1 + row2) / 2)
        
        # Making sure the midrow does not go over bound
        midrow = max(0, min(midrow, self.row - 1))

        # Get midrow width and Horizontal distance (east/west direction)
        midrow_width = self.meters_per_x_lookup[midrow]
        dist_x = (col2 - col1) * midrow_width
        
        return math.sqrt(dist_x ** 2 + dist_y ** 2)
    
    def _get_neighbors(self, loc_idx: int) -> list:
        """
        Find all 8 neighbor (direct-contact) pixels around the central pixel. 
        Append them to neighbors list if in range of the dem and output the list.

        Args:
            - node: the centre node that is the base of neighbor-searching
        
        Return:
            - neighbors: the list that stores all the row and col of the surrounding neighbors.
        """
        row, col = divmod(loc_idx, self.col)
        
        neighbors = []

        node_directions = [(-1, 0), (1, 0), (-1, 1), (1, 1), (-1, -1), (1, -1), (0, -1), (0, 1)]
        
        for r, c in node_directions:
            new_rol, new_col = row + r, col + c

            if 0 <= new_rol < self.row and 0 <= new_col < self.col:
                neighbors.append(new_rol * self.col + new_col)

        return neighbors
    
    def _get_movement_cost(self, current_idx: int, neighbor_idx: int) -> float:

        # Convert index param to tuple(int, int)
        current = divmod(current_idx, self.col)
        neighbor = divmod(neighbor_idx, self.col)
        
        dist_cost = self._heuristic(current_idx, neighbor_idx)

        curr_elev = self.dem[current]
        neigh_elev = self.dem[neighbor]

        # Check for no data, usually represented by a large negative number
        if curr_elev <= -100 or neigh_elev <= -100:
            return float("inf") # cannot fly path that no data area (uncertain of the terrain)
        

        # Decision penalty

        climb_height = neigh_elev - curr_elev
        

        # Situation 1: downhill (go down valley, prefered, less penalty)
        if climb_height < 0:
            return dist_cost + (climb_height * 0.5)

        # Situation 2: uphill (hill, mountain, those requires to go up)
        gradient = climb_height / (dist_cost + 1e-6) # +1e-6 to prevent div by zero error
        penalty = 0.0
        
        # Level 1: gentle slope (<3°)
        if gradient <= 0.05:
            # ignore, penalty weight: low (scale of 2)
            penalty = climb_height * 2.0

        # Level 2: moderate (3-8.5°)
        elif gradient <= 0.15:
            # penalty weight: medium (10)
            penalty = climb_height * 10.0

        # Level 3: steep (>8.5°)
        elif gradient > 0.15:
            # penalty weight: high (100), the program will chose other path unless still the closest even with penalty
            penalty = climb_height * 100.0 # apply a lot of penalty to very steep 

        return dist_cost + penalty
    

    def pathfinding(self, start: tuple[int, int], end: tuple[int, int], heuristic_weight: float=1.1) -> None:
        """
        
        Note:
            The pathfining algorithm will implement multiple approach to improve RAM usage, so device with less RAM can still execute the program.
        Args:
            - 
        """

        # if heuristic_weight < 1:
        #     raise ValueError("heuristic_weight cannot be less than 1")
        # Turn start, end pixel coord tuple into int index to save memory
        start_idx = start[0] * self.col + start[1]
        end_idx = end[0] * self.col + end[1] 
        
        # Initialise some crucial storage
        came_from = {}
        open_set = []
        heapq.heappush(open_set, (0, start_idx)) # open_set: (f_score, pixel_idx)

        # Initiate the dict for g_score
        g_score = {start_idx: 0.0}

        current_loc = ()
        
        # Count how many nodes were explored
        node_explored = 0

        print(f"Pathfinding: {start} -> {end} | Weight: {heuristic_weight}")

        while open_set:

            node_explored += 1 

            current_f, current_idx = heapq.heappop(open_set)
            
            # Do the index -> row, col pixel conversion to convert index back to pixel coordinate
            current_row, current_col = divmod(current_idx, self.col)
            current_loc = (current_row, current_col)

            # Check if target reached
            if current_idx == end_idx:
                print(f"Target acquired, pathfinding done... Total of {node_explored} nodes explored.")
                return self._reconstruct_path(came_from, current_idx)

            for neigh_idx in self._get_neighbors(current_idx):

                 
                # Obtain movement cost
                move_cost = self._get_movement_cost(current_idx, neigh_idx)

                if move_cost == float("inf"): # if "no data" area, proceeds
                    continue
                
                temp_g_score = g_score[current_idx] + move_cost
                
                # Discover better path and update to open_set for each nighbor
                if temp_g_score < g_score.get(neigh_idx, float("inf")):
                    # store the neighbor node into came_from (so we remember the path)
                    came_from[neigh_idx] = current_idx
                    g_score[neigh_idx] = temp_g_score

                    # get f_score (find h_score first)
                    h_score = self._heuristic(neigh_idx, end_idx)
                    f_score = temp_g_score + (h_score * heuristic_weight)

                    # push to open_set priority queue
                    heapq.heappush(open_set, (f_score, neigh_idx))
        
        print("Queue exhausted. No path found.")
        return None

    def _reconstruct_path(self, came_from: dict, current_index: int=0) -> list:
        """
        
        """
        path = []
        
        while current_index in came_from:
            path.append(divmod(current_index, self.col))
            # the current index is set to the item, which corresponds to the parent
            current_index = came_from[current_index]

        # Add the start node
        path.append(divmod(current_index, self.col))

        # Reverse the list: [End, B, A, Start] -> [Start, A, B, End]
        return path[::-1]
    

    # ------ Test Verification Functions ------
    def get_surfcae_distance(self, loc1: tuple[float, float], loc2: tuple[float, float]) -> float:
        """
        Get the distance (ground distance, ignoring height) of two GPS points. Take into consideration of shrink of latitude shrink.
        Mainly using the Haversine distance formula to acheive the purpose.

        Args:
            - loc1 / loc2: tuple that stores the lat/lon coordinate
        """

        lat1, lon1 = loc1
        lat2, lon2 = loc2

        # Convert to radians
        lat1_r, lon1_r = math.radians(lat1), math.radians(lon1)
        lat2_r, lon2_r = math.radians(lat2), math.radians(lon2)

        dlon = lon2_r - lon1_r
        dlat = lat2_r - lat1_r

        a = math.sin(dlat / 2) ** 2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2) ** 2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371000 # Earth radius in meter
        
        return c * r
    
    def idx_to_pixel(self, index):
        """
        Docstring for idx_to_pixel
        
        :param self: Description
        :param index: Description
        """
        row, col = divmod(index, self.col)
        pixel = (row, col)
        return pixel
    
    def pixel_to_idx(self, row, col):
        return row * self.col + col
    # ------ Test Ends ------