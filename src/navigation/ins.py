import numpy as np
import math


class INS:
    def __init__(self, init_pos: np.ndarray, init_vel: np.ndarray, init_att: np.ndarray):
        """
        Initializes the Inertial Navigation System.

        Args:
            init_pos: [x, y, z] in meters
            init_vel: [vx, vy, vz] in m/s
            init_att: [pitch, roll, heading] in radians
        """
        # Current Estimated State
        self.pos = init_pos.astype(float)
        self.vel = init_vel.astype(float)
        self.att = init_att.astype(float)

        # Typical INS sensor bias/drift characteristics (Simplified)
        self.accel_bias = np.array([0.01, 0.01, 0.05])  # m/s^2
        self.gyro_drift = np.array([0.001, 0.001, 0.001])  # rad/s

    def update(self, dt: float, accel_body: np.ndarray, gyro_rates: np.ndarray):
        """
        Propagates the navigation state using inertial sensors.

        Args:
            dt: Time step in seconds
            accel_body: Linear acceleration in body frame
            gyro_rates: Angular velocities
        """
        # 1. Apply Bias and Noise (Simulating sensor imperfection)
        # In a real simulation, you would add Gaussian noise here
        true_accel = accel_body - self.accel_bias
        true_gyro = gyro_rates - self.gyro_drift

        # 2. Update Attitude (Simple Euler Integration)
        self.att += true_gyro * dt
        self.att[2] %= (2 * math.pi)  # Normalize heading

        # 3. Transform Acceleration from Body Frame to World Frame
        # Simplified: Assuming heading-only rotation for basic XY propagation
        theta = self.att[2]
        world_accel = np.array([
            true_accel[0] * math.cos(theta) - true_accel[1] * math.sin(theta),
            true_accel[0] * math.sin(theta) + true_accel[1] * math.cos(theta),
            true_accel[2] - 9.80665  # Subtract gravity for Z-axis
        ])

        # 4. Integrate Velocity and Position
        self.vel += world_accel * dt
        self.pos += self.vel * dt

    def get_estimate(self) -> tuple:
        """Returns the current dead-reckoning position and velocity."""
        return self.pos, self.vel

    def correct_state(self, corrected_pos: np.ndarray, corrected_vel: np.ndarray):
        """
        Used by the Kalman Filter to reset INS drift.
        """
        self.pos = corrected_pos
        self.vel = corrected_vel