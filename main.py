import sys
import time
import math
import re
import json
import numpy as np
from pathlib import Path
import fastplotlib as fpl
import matplotlib.pyplot as plt

from src.missile.state import MissileState
from src.missile.profile import MissileProfile
# from src.navigation.system import NavigationSystem
from src.guidance.pathfinding_backend import Pathfinding
from src.guidance.trajectory import TrajectoryGenerator
from src.terrain.dem_loader import DEMLoader
from src.visualization.plotter import MissionPlotter
from src.control.timer import InternalTimer


# --- Data Constants (missile profile) ---
LAUNCH_PASSWORD = 0000

TOMAHAWK_BLOCK_V = {
    # km/h is used instead of m/s and deg instead of radian for better understanding,
    # will be converted back into m/s in data handling
    "cruise_speed": 800,         # km/h
    "min_speed": 400,            # km/h
    "max_speed": 920,            # m/s
    "max_acceleration": 9.8,     # m/s^2
    "min_altitude": 30,          # m (AGL)
    "max_altitude": 1200,        # m (AGL)
    "max_g_force": 6.89,         # g-force
    "sustained_turn_rate": 8.0,  # deg/s
    "sustained_g_force": 2.0,    # g-force
    "evasive_turn_rate": 25.0,   # deg/s
}

# ----------------------------------------

# --- Helper Functions (for simulation) ---
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

def parse_custom_profile(current_data):
    print_header("Custom Missile Configuration")
    profile = current_data.copy()
    profile["name"] = "Custom Configuration"

    print("   [INFO] Press ENTER to keep default values.\n")
    print("   -------------------------------------------")

    try:
        for key, default in profile.items():
            if key == "name": continue

            # Replace _ in label with space for readability
            label = key.replace('_', ' ').title()
            unit = ""
            # Basically detect keywords in keys and set corresponding unit (user-end)
            if "speed" in key: unit = "(km/h)"
            elif "rate" in key: unit = "(deg/s)"
            elif "altitude" in key: unit = "(m)"
            elif "force" in key: unit = "(G)"

            val = input(f"   >> {label} {unit} [{default}]: ").strip()
            if val:
                profile[key] = float(val)

    except ValueError:
        # Opps! (tryna keep as professional as possible)
        print("\n      [!] Invalid number entered. Aborting changes.")
        time.sleep(1)
        return None
    return profile


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
# -----------------------------------------

# --- Launchpad Menu Display / Data Handling ---
def clear_screen(): print("\n" * 5) # Create blank screen for clearer view

def print_header(title): print(f"\n{'='*65}\n   {title.upper()}   \n{'='*65}\n") # Print header to console
def prompt_gps_loop(prompt_text):
    """
    UI Wrapper that loops until valid input is received using the
    existing 'parse_gps_input' function.
    """
    while True:
        raw = input(f"   >> {prompt_text} (or 'b' to back): ").strip()

        if raw.lower() == 'b':
            return None

        try:
            # USE EXISTING FUNCTION HERE
            return parse_gps_input(raw)
        except ValueError:
            print("      [!] Format Error. Accepted formats:")
            print("          - 22.5, 120.3")
            print("          - 22.5 N, 120.3 E")

def create_missile_profile(data: dict) -> MissileProfile:
    """
    Receive the dictionary data (for example, like the TOMAHAWK_BlOCK_V you can see from top.
    Convert the unit km/h to standard m/s, degree to radian, then return as MissileProfile object.

    Arg:
        - data: a dictionary that contains each parameters for MissileProfile object.
    """

    KMH_TO_MS = 1.0 / 3.6
    DEG_TO_RAD = math.pi / 180.0

    return MissileProfile(
        cruise_speed = data["cruise_speed"] * KMH_TO_MS,
        min_speed = data["min_speed"] * KMH_TO_MS,
        max_speed = data["max_speed"] * KMH_TO_MS,
        max_acceleration = data["max_acceleration"],
        min_altitude = data["min_altitude"],
        max_altitude = data["max_altitude"],
        max_g_force = data["max_g_force"],

        sustained_turn_rate = data["sustained_turn_rate"] * DEG_TO_RAD,
        sustained_g_force = data["sustained_g_force"],

        evasive_turn_rate = data["evasive_turn_rate"] * DEG_TO_RAD,
    )

# ----------------------------------------------

class MissileSimulation:
    def __init__(self, start_gps: tuple, target_gps: tuple, dem_name: str, profile: MissileProfile) -> None:
        """
        Subsystem initialisation
        """
        self.start_gps = start_gps
        self.target_gps = target_gps
        self.dem_name = dem_name
        self.profile = profile
        
        # Construct full path to DEM file
        from pathlib import Path
        dem_path = Path(__file__).parent / "data" / "dem" / self.dem_name

        self.dem_loader = DEMLoader(dem_path)

        init_z = self.dem_loader.get_elevation(*self.start_gps)

        # Initiate pathfinding through Pyhton backend (as an intermediate between CPP and simulation)
        self.pf = Pathfinding(self.dem_name)
        self.traj_gen = TrajectoryGenerator(self.pf.engine, self.pf.dem_loader)

        # Generate actual path (with pixels from pathfinding)
        start_pixel = self.dem_loader.lat_lon_to_pixel(*self.start_gps)
        target_pixel = self.dem_loader.lat_lon_to_pixel(*self.target_gps)

        # Run the pathfinder and save it to raw_path
        print(f"   [INFO] Planning route...")
        raw_path = self.pf.find_path(start_pixel, target_pixel)

        if raw_path:
            self.trajectory = self.traj_gen.get_trajectory(raw_path)
        else:
            self.trajectory = []

        # Define initial state
        self.missile_state = MissileState(x=0, y=0, z=init_z,
                                    vx=0, vy=0, vz=0,
                                    pitch=0, roll=0, heading=0,
                                    lon=self.start_gps[0], lat=self.start_gps[1], altitude=init_z,
                                    time=0, gps_valid=True, tercom_active=False, ins_calibrated=False,
                                    distance_traveled=0, distance_to_target=get_path_distance(self.trajectory))

        dist_meters = get_path_distance(self.trajectory)
        print(f"   [INFO] Path Length: {dist_meters / 1000:.2f} km")

        # Conversion Factor for Plotting (Lat/Lon -> Meters X/Y)
        self.meters_per_lat = 111320.0
        self.meters_per_lon = 111320.0 * math.cos(math.radians(self.start_gps[0]))

    def plot(self):
        """
        Visualisation generated by Gemini, verified personally.
        """
        print(f"\n   >>> VISUALIZING TRAJECTORY")
        print(f"   >>> TARGET: {self.target_gps}")

        if not self.trajectory:
            print("   [ERROR] Cannot plot empty trajectory.")
            return

        # --- 1. CONVERT PATH TO METERS ---
        # Plotter needs X/Y in meters (relative to start)
        local_path_meters = []
        for pt in self.trajectory:
            lat, lon, alt = pt
            d_lat = lat - self.start_gps[0]
            d_lon = lon - self.start_gps[1]
            x = d_lon * self.meters_per_lon
            y = d_lat * self.meters_per_lat
            local_path_meters.append([x, y, alt])

        # --- 2. PREPARE STATIC DATA ---
        # We assume the missile follows the path exactly for this visualization
        history = {
            'time': list(range(len(local_path_meters))),  # Dummy time axis
            'true_pos': local_path_meters,
            'est_pos': local_path_meters  # No error shown
        }

        # --- 3. OPEN PLOTTER ---
        print("   [INFO] Opening Plot Window...")
        try:
            MissionPlotter.plot_mission(
                history,
                planned_path=local_path_meters,
                dem_file=self.dem_name,
                origin=self.start_gps,
                title=f"Planned Trajectory Analysis (under development)",
                downsample=2
            )
        except Exception as e:
            print(f"   [PLOT ERROR] {e}")
            import traceback
            traceback.print_exc()

    def run(self):
        InternalTimer().start()




# --- MAIN ENTRY POINT (Gemini Assisted piece of junk) ---
# i seriously hate ui, visualisation...
if __name__ == "__main__":

    # 1. System Configuration State
    config = {
        "dem_name": "merged_dem_sib_N54_N59_E090_E100.tif",
        "start_gps": None,
        "target_gps": None,
        "missile_profile": TOMAHAWK_BLOCK_V.copy()
    }

    while True:
        clear_screen()
        print_header("Cruise Missile Simulation Console - Main Menu")

        # Format status strings (GPS coordinates) for display
        s_gps_str = f"{config['start_gps']}" if config['start_gps'] else "[ NOT SET ]"
        t_gps_str = f"{config['target_gps']}" if config['target_gps'] else "[ NOT SET ]"

        # Main menu options (level 0)
        print(f"   1. Set Terrain File      [{config['dem_name']}]")
        print(f"   2. Set Coordinates       [Start: {s_gps_str}]")
        print(f"                            [Target: {t_gps_str}]")
        print(f"   3. Missile Configuration [{config['missile_profile']}]")
        print("   -----------------------------------------------------------------")
        print("    4. Calculate Path ")
        print(f"   5. INITIALIZE & LAUNCH")
        print(f"   6. Exit Console")
        print("\n" + "=" * 65)

        choice = input("\n   >> Select Option [1-5]: ").strip()

        # --- OPTION 1: SET DEM ---
        if choice == '1':
            print_header("Terrain Configuration")
            print(f"   Current File: {config['dem_name']}")
            print("   Enter new filename (must be located in src/data/dem/).")

            new_name = input("\n   >> Filename (ENTER to keep current): ").strip()
            if new_name:
                config['dem_name'] = new_name
                print("      [OK] Terrain filename updated.")
            else:
                print("      [INFO] No change made.")
            time.sleep(1)

        # --- OPTION 2: SET COORDINATES ---
        elif choice == '2':
            while True:
                clear_screen()
                print_header("Coordinate Configuration")

                # If start_gps is none, print "NOT SET"
                curr_start = config['start_gps'] if config['start_gps'] else "NOT SET"
                curr_target = config['target_gps'] if config['target_gps'] else "NOT SET"

                print(f"   1. Set Start Location   [{curr_start}]")
                print(f"   2. Set Target Location  [{curr_target}]")
                print(f"   3. Return to Main Menu")
                print("\n" + "=" * 65)

                sub_choice = input("\n   >> Select Option [1-3]: ").strip()

                # You gotta know where you are and where's your target right?
                if sub_choice == '1':
                    print("\n   --- Input Launch Site ---")
                    # Loops until valid using existing parse function, or user simply rage quit
                    res = prompt_gps_loop("Enter GPS")
                    if res: config['start_gps'] = res

                elif sub_choice == '2':
                    print("\n   --- Input Target Site ---")
                    res = prompt_gps_loop("Enter GPS")
                    if res: config['target_gps'] = res

                elif sub_choice == '3':
                    break

        # --- OPTION 3: MISSILE PROFILE ---
        elif choice == '3':
            while True:
                data = config["missile_profile"]
                clear_screen()
                print_header("Missile Profile Selection")
                print(f"   Current Profile: {config['missile_profile']}\n")

                print("   1. Add Cusstom Profile")
                print("   2. Load Presets")
                print("   3. Return to Main Menu")
                print("\n" + "=" * 65)

                sub_choice = input("\n   >> Select Option [1-3]: ").strip()

                if sub_choice == '1':
                    custom_data = parse_custom_profile(data)

                    if custom_data:
                        config['missile_profile'] = custom_data
                        print("\n      [OK] Custom Profile Applied.")

                elif sub_choice == '2':
                    print_header("Presets Selection (Note: Based On Public Data Only)")
                    print("   Due to lack of funding, we only got Tomahawk Block V available.")
                    print("   Agent 008 is on his way to steal some other missiles...")
                    config['missile_profile'] = TOMAHAWK_BLOCK_V.copy()
                    print("\n      [OK] Preset Loaded.")
                    time.sleep(1)
                    break
                elif sub_choice == '3':
                    break

        # --- OPTION 4: PATHFINDING & PLOT ---
        elif choice == '4':
            # Pre-flight checks
            if not config['start_gps'] or not config['target_gps']:
                print("\n      [ERROR] Aborted: Start and Target coordinates are required.")
                input("      Press ENTER to continue...")
                continue

            print_header("Mission Check (please validates mission info before starting plotter)")
            print(f"   Terrain: {config['dem_name']}")
            print(f"   Profile: {config['missile_profile']}")
            print(f"   Launch:  {config['start_gps']}")
            print(f"   Target:  {config['target_gps']}")
            print("-" * 65)

            confirm = input("\n   >> Start Plotter? (y/n): ").strip().lower()
            if confirm == 'y':
                try:
                    print("\n   [INFO] Initializing Simulation Environment...")

                    active_profile = create_missile_profile(config['missile_profile'])

                    # Instantiate the main simulation class (passing arguments)
                    sim = MissileSimulation(
                        config['start_gps'],
                        config['target_gps'],
                        config['dem_name'],
                        active_profile
                    )

                    print("   [INFO] Plotting Path...")
                    sim.plot()

                    input("\n   [COMPLETE] Plotting Finished. Press ENTER for Menu...")
                except Exception as e:
                    print(f"\n   [FATAL ERROR] Plotter Failed: {e}")
                    # Optional: Print full traceback for debugging
                    import traceback

                    traceback.print_exc()
                    input("   Press ENTER to return to menu...")
            else:
                print("      [INFO] Plotter Aborted.")
                time.sleep(1)

        # --- Option 5: LAUNCH ---
        elif choice == '5':
            # Check launch password (just for fun)
            pass_check = input("\n   >> Please Enter Launch Password (optional)")
            print("PASSWORD CONFIRMED")

            confirm = input("\n   >> Confirm Launch Sequence? (y/n): ").strip().lower()

            if confirm == 'y':
                print("COUNTING DOWN... THERE'S NO RETURN")
                for i in range(3, 0, -1):
                    print(i)
                    time.sleep(1)

                sim.run()


        # --- OPTION 6: QUIT ---
        elif choice == '6':
            print("\n   [INFO] Terminating Session. Goodbye.\n")
            sys.exit()