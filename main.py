import sys
import time
import math
import re
import numpy as np
from pathlib import Path
import fastplotlib as fpl
import matplotlib.pyplot as plt


from src.missile.state import MissileState
# from src.navigation.system import NavigationSystem
from src.guidance.pathfinding_backend import Pathfinding
from src.guidance.trajectory import TrajectoryGenerator
from src.terrain.dem_loader import DEMLoader
from src.visualization.plotter import MissionPlotter

def parse_gps_input(gps_input: str) -> tuple[float, float]:
    """
    Parses a GPS string into (latitude, longitude) floats.
    Supported formats:
        - "22.5, -120.3      (Comma separated, +/-)"
        - "22.5 -120.3       (Space separated, +/-)"
        - "22.5 N, 120.3 W   (Directional, NSWE)"

    We then check if the user chose to input with NSWE for directions, if so, then we turn into +/- format for
    rasterio to recognize our format.

    Args:
        - gps_input: GPS string to parse
    """

    clean_str = gps_input.strip().upper().replace(",", " ") # unifying format, remove space, comma, and turn upper case

    # Extract float-like numbers
    numbers = [float(x) for x in re.findall(r'-?\d+\.?\d*', clean_str)]

    if len(numbers) != 2:
        raise ValueError(f"Could not find exactly 2 coordinates for the input{gps_input}. ")

    lat, lon = numbers[0], numbers[1]

    # Check for 'S' (South) -> Negate Latitude
    if 'S' in clean_str:
        lat = -abs(lat)
    # Check for 'N' (North) -> Positive Latitude
    if 'N' in clean_str:
        lat = abs(lat)

    # Check for 'W' (West) -> Negate Longitude
    if 'W' in clean_str:
        lon = -abs(lon)
    # Check for 'E' (East) -> Positive Longitude
    if 'E' in clean_str:
        lon = abs(lon)

    return lat, lon

def get_path_distance(trajectory: list[float, float, float]) -> float:
    """
    We cannot calculate flight distance by just counting trajectory points or multiplies total pixels by x meter
    grid resolution (30 for our sample dem file). The B-Spline smoothing algorithm creates a variable density of
    non-equidistant waypoints. It clusters them on curves and spreads them on straight sections. This separates the array
    length from physical metric distance.

    Instead, we calculate haversine distance for every neighbouring points on the trajectory
    then sum up.

    Return:
        - distance in meter
    """

    if not trajectory or len(trajectory) < 2:
        return 0.0

    total_dist = 0.0
    r = 637100 # Earth radius in meter

    # Main loop for iterating and accumulating total distance
    for i in range(len(trajectory) - 1):

        lat1, lon1 = trajectory[i][0], trajectory[i][1]
        lat2, lon2 = trajectory[i+1][0], trajectory[i+1][1]

        # Convert to radians
        lat1_r, lon1_r = math.radians(lat1), math.radians(lon1)
        lat2_r, lon2_r = math.radians(lat2), math.radians(lon2)

        dlon = lon2_r - lon1_r
        dlat = lat2_r - lat1_r

        a = math.sin(dlat / 2) ** 2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2) ** 2
        c = 2 * math.asin(math.sqrt(a))

        total_dist += r * c

        return total_dist

class MissileSimulation:
    def __init__(self, start_gps: tuple, target_gps: tuple, dem_name: str):
        """
        Subsystem initialisation
        """
        self.start_gps = start_gps
        self.target_gps = target_gps
        self.dem_name = dem_name

        self.dem_loader = DEMLoader(self.dem_name)

        init_z = self.dem_loader.get_elevation(*self.start_gps)

        # Initiate pathfinding through Pyhton backend (as an intermediate between CPP and simulation)
        self.pf = Pathfinding("self.dem_name")
        self.trajectory = TrajectoryGenerator(self.pf.engine, self.pf.dem_loader)

        # Missile start at x = 0, y = 0, z = start location's altitude
        self.missile = MissileState(x=0, y=0, z=init_z,
                                    vx=0, vy=0, vz=0,
                                    pitch=0, roll=0, heading=0,
                                    lon=self.start_gps[0], lat=self.start_gps[1], altitude=init_z,
                                    time=0, gps_valid=True, tercom_active=False, ins_calibrated=False,
                                    distance_traveled=0, distance_to_target=get_path_distance(self.trajectory))


    def run(self):
        pass


if __name__ == "__main__":
    print("--- Missile Simulation ---")

    # Default sample DEM: Siberia, N54 ~ N59, E90 ~ E100
    default_dem = "merged_dem_sib_N54_N59_E090_E100.tif"
    dem_name = input("Please enter the name of the dem file (optional): ")

    if not dem_name:
        dem_name = default_dem

    if ".tif" not in dem_name:
        dem_name += ".tif"

    # Handle coordinate input & starting simulation
    try:
        print("Supported formats: "
                "22.5, -120.3      (Comma separated, +/-)"
                "22.5 -120.3       (Space separated, +/-)"
                "22.5 N, 120.3 W   (Directional (NSWE))")

        raw_start = input("Enter start GPS location: ")
        start_gps = parse_gps_input(raw_start)

        raw_target = input("Enter target GPS location: ")
        target_gps = parse_gps_input(raw_target)

        sim = MissileSimulation(start_gps, target_gps, dem_name)


    except ValueError as e:
        print(e)
        print("Please enter coordinate following the format stated above")







        

