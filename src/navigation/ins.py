import numpy as np
import math

from ..terrain.dem_loader import DEMLoader

class INS:
    def __init__(self, init_pos: list[float, float, float], init_v: list[float, float, float], init_att) -> None:
        """
        Args:
            - init_pos: Initial position as a list, [x, y, z]
            - init_v: Initial velocity as a list, [vx, vy, vz]
            - init_alt: Initial attitude as a list, [pitch, roll, heading] in radian 
        """

    pass