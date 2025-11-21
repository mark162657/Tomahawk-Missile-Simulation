from ..terrain.dem_loader import DEMLoader
import math
from pathlib import Path
import numpy as np

class Pathfinding:
    """
    Using A*pathfinding algorithm to find the most ideal path that considers the horizontal and vertical movement.
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

    # *** FOR TESTING ONLY ***
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

    def heuristic(self, loc1: tuple[int, int], loc2: tuple[int, int]) -> float:
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
        row1, col1 = loc1
        row2, col2 = loc2

        # Get vertical distance (north/south direction)
        dist_y = (row2 - row1) * self.meter_per_y

        # Midrow between two locations
        midrow = int((row1 + row2) / 2)
        
        # Making sure the midrow does not go over bound
        midrow = max(0, min(midrow, self.row - 1))

        # Get midrow width and Horizontal distance (east/west direction)
        midrow_width = self.meters_per_x_lookup[midrow]
        dist_x = (col2 - col1) * midrow_width
        
        return math.sqrt(dist_x ** 2 + dist_y ** 2)
    
    def _get_neighbors(self, node: tuple[int, int]) -> list:
        """
        Find all 8 neighbor (direct-contact) pixels around the central pixel. 
        Append them to neighbors list if in range of the dem and output the list.

        Args:
            - node: the centre node that is the base of neighbor-searching
        
        Return:
            - neighbors: the list that stores all the row and col of the surrounding neighbors.
        """
        row, col = node
        
        neighbors = []

        node_directions = [(-1, 0), (1, 0), (-1, 1), (1, 1), (-1, -1), (1, -1), (0, -1), (0, 1)]
        
        for r, c in node_directions:
            new_rol, new_col = row + r, col + c

            if 0 <= new_rol < self.row and 0 <= new_col < self.col:
                neighbors.append((new_rol, new_col))

        return neighbors
    
    def _get_movement_cost(self, current: tuple[int, int], neighbor: tuple[int, int]):
        dist_cost = self.heuristic(current, neighbor)

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
        
        # Level 1: gentle slope
        if gradient <= 0.05:
            # ignore, penalty weight: low (scale of 5)
            penalty = climb_height * 5.0

        # Level 2: a bit steep slope
        elif gradient <= 0.15:
            # penalty weight: medium (20)
            penalty = climb_height * 20

        # Level 3: other steep slope
        elif gradient > 0.15:
            # penalty weight: high (100), the program will chose other path unless still the closest even with penalty
            penalty = climb_height * 100 # apply a lot of penalty to very steep 

        return dist_cost + penalty


