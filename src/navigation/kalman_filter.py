import numpy as np
import time

class KalmanFilter:
    def __init__(self, dt, ux, uy, uz, std_noise, std_mea):

        # Sampling time
        self.dt = dt

        # Control input variable matrix
        # Initial control input (acceleration), implies [ax, ay, az]
        self.u = np.matrix([ux, uy, uz])
        
        # State transition matrix (A) 6 * 6 for 3D
        # State: [x, y, z, vx, vy, vz]
        # x_new = x + vx * dt ...
        self.A = np.eye(6) 
        self.A[0, 3] = self.dt
        self.A[1, 4] = self.dt
        self.A[2, 5] = self.dt

        # Control input matrix (B) 6 * 3 for 3D
        # Affects position (0.5*a*t^2) and velocity (a*t)
        self.B = np.zeros((6, 3))

        # 1/2 * dt ** 2 (Position update from acceleration)
        self.B[0, 0] = 0.5 * self.dt ** 2
        self.B[1, 1] = 0.5 * self.dt ** 2
        self.B[2, 2] = 0.5 * self.dt ** 2
        
        # dt (Velocity update from acceleration)
        self.B[3, 0] = self.dt
        self.B[4, 1] = self.dt
        self.B[5, 2] = self.dt

        # Measurement Matrix (H) - We measure position (x, y, z) only, not velocity
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1

        # Process Noise Covariance (Q)
        q_element = std_noise ** 2
        self.Q = np.eye(6) * q_element

        # Measurement Noise Covariance (R)
        r_element = std_mea ** 2
        self.R = np.eye(3) * r_element

        # Initial Error Covariance (P)
        self.P = np.eye(6)

        # Initial State (x) [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]
        self.x = np.zeros((6, 1))

        # Ensure u is column vector (3, 1)
        if self.u.shape == (1, 3):
            self.u = self.u.T

    def predict(self, u_acc=None):
        """
        Predict future state based on physics model (A) and control input (u).
        u_acc: Optional tuple (ax, ay, az). If None, uses self.u initialized in __init__
        """
        if u_acc:
             self.u = np.matrix(u_acc).T # Transpose to column vector

        # x = A * x + B * u
        self.x = self.A @ self.x + self.B @ self.u

        # P = A * P * A.T + Q
        self.P = self.A @ self.P @ self.A.T + self.Q
        
        return self.x

    def update(self, z_meas):
        """
        Correct predicted state with actual measurement data.
        z_meas: tuple or list [x, y, z] from sensors (GPS, Barometer)
        """
        z = np.matrix(z_meas).T # Transpose to column vector

        # S = H * P * H.T + R
        S = self.H @ self.P @ self.H.T + self.R

        # K = P * H.T * inv(S)
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # x = x + K * (z - H * x)
        y = z - (self.H @ self.x) # Innovation (Measurement Residual)
        self.x = self.x + (K @ y)

        # P = (I - K * H) * P
        I = np.eye(self.H.shape[1])
        self.P = (I - (K @ self.H)) @ self.P

        return self.x.flatten().tolist()[0]