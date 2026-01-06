class TERCOM:
    """
    TERCOM (Terrain Contour Matching): A navigation system that guides missiles by
    comparing ground elevation profiles measured by radar altimeter with pre-stored
    terrain maps to determine position and correct flight path. Implemented by Kalman Filter.

    The terrain check will be performed every 5 seconds. With following data:
        - db data: from TerrainDatabase.get_elevation_patch
        - sensor data:
    """

    def __init__(self):
        pass