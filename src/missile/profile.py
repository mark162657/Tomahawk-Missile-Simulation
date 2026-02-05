from dataclasses import dataclass

@dataclass
class MissileProfile:
    """
    Unit:
        speed: m/s
        distance: m
        altitude: m (AGL)
        acceleration: m/s^2
        time: second
        angle: radian
        turn rate: radian/second
        mass: kg
    """
    cruise_speed: float
    min_speed: float
    max_speed: float
    max_acceleration: float
    min_altitude: float
    max_altitude: float
    max_g_force: float

    # missile have two maneuver types:
    # sustained: log-g sustainable maneuver
    sustained_turn_rate: float
    sustained_g_force: float

    # evasive jink (high-g evasion)
    evasive_turn_rate: float

    def calculate_turning_radius(self, speed: float, turn_rate: float) -> float:
        """
        Convert turn rate to turning radius. By formula: r = velocity / turn rate

        Args:
            - speed: current speed m/s
            - turn_rate: current turn rate

        Return:
            - turning radius (m): prevent km / m unit confusion
        """
        if abs(turn_rate) < 1e-6: # prevent divide by 0 error, take into consideration of float point error
            return float("inf")
        
        return speed / turn_rate

    def get_max_lateral_acceleration(self) -> float:
        """
        Get max lateral acceleration based on max g-force

        Return:
            - lateral acceleration (m/s^2)
        """
        g = 9.80665
        return g * self.max_g_force

    def validate_maneuver (self, current_speed: float, desired_speed: float, turn_rate: float) -> bool:
        # Check speed limits
        if not (self.min_speed <= current_speed <= self.max_speed):
            return False

        # Check turn rates
        max_allowed_turn_rate = max(self.sustained_turn_rate, self.sustained_g_force)
        if turn_rate > max_allowed_turn_rate:
            return False

        # Check and compute acceleration for speed change
        acceleration_required = abs(desired_speed - current_speed)
        if acceleration_required > self.max_acceleration:
            return False

        # Check if turn rate is greater than evasive turn rate
        if turn_rate > self.evasive_turn_rate:
            return False

        # Compute lateral acceleration
        lateral_acceleration = desired_speed * turn_rate

        max_lateral_acceleration = self.get_max_lateral_acceleration()

        if lateral_acceleration > max_lateral_acceleration:
            return False

        return True


    def get_turn_rate_for_maneuver(self, maneuver_type: str) -> float:
        if maneuver_type.lower() == "manual":
            return self.sustained_turn_rate
        elif maneuver_type.lower() == "evasive":
            return self.evasive_turn_rate
        else:
            raise ValueError("Unknown maneuver type")
    






