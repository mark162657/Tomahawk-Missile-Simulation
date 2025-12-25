'''
    File name         : KalmanFilter.py
    Description       : KalmanFilter class used for object tracking
    Author            : Rahmad Sadli
    Date created      : 20/02/2020
    Python Version    : 3.7
'''

import numpy as np
import matplotlib.pyplot as plt

class KalmanFilter(object):
    def __init__(self, dt, u_x,u_y, std_acc, x_std_meas, y_std_meas):
        """
        :param dt: sampling time (time for 1 cycle)
        :param u_x: acceleration in x-direction
        :param u_y: acceleration in y-direction
        :param std_acc: process noise magnitude
        :param x_std_meas: standard deviation of the measurement in x-direction
        :param y_std_meas: standard deviation of the measurement in y-direction
        """

        # Define sampling time
        self.dt = dt

        # Define the  control input variables
        self.u = np.matrix([[u_x],[u_y]])

        # Intial State
        self.x = np.matrix([[0], [0], [0], [0]])

        # Define the State Transition Matrix A
        self.A = np.matrix([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        # Define the Control Input Matrix B
        self.B = np.matrix([[(self.dt**2)/2, 0],
                            [0,(self.dt**2)/2],
                            [self.dt,0],
                            [0,self.dt]])

        # Define Measurement Mapping Matrix
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        #Initial Process Noise Covariance
        self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
                            [0, (self.dt**4)/4, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, self.dt**2, 0],
                            [0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2

        #Initial Measurement Noise Covariance
        self.R = np.matrix([[x_std_meas**2,0],
                           [0, y_std_meas**2]])

        #Initial Covariance Matrix
        self.P = np.eye(self.A.shape[1])

    def predict(self):
        # Refer to :Eq.(9) and Eq.(10)  in https://machinelearningspace.com/object-tracking-simple-implementation-of-kalman-filter-in-python/?preview_id=1364&preview_nonce=52f6f1262e&preview=true&_thumbnail_id=1795

        # Update time state
        #x_k =Ax_(k-1) + Bu_(k-1)     Eq.(9)
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        # Calculate error covariance
        # P= A*P*A' + Q               Eq.(10)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x[0:2]

    def update(self, z):

        # Refer to :Eq.(11), Eq.(12) and Eq.(13)  in https://machinelearningspace.com/object-tracking-simple-implementation-of-kalman-filter-in-python/?preview_id=1364&preview_nonce=52f6f1262e&preview=true&_thumbnail_id=1795
        # S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  #Eq.(11)

        self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))   #Eq.(12)

        I = np.eye(self.H.shape[1])

        # Update error covariance matrix
        self.P = (I - (K * self.H)) * self.P   #Eq.(13)
        return self.x[0:2]



import numpy as np

class KalmanFilter:
    def __init__(self, dt, process_noise_std, meas_std_val, initial_state=None):
        """
        Initializes the 6-State Kalman Filter for 3D navigation.
        
        Args:
            dt (float): Time step in seconds (e.g. 0.1)
            process_noise_std (float): Trust in physics/INS (Standard Deviation). 
                                       Higher = Trust Physics Less.
            meas_std_val (tuple): (x_std, y_std, z_std) - Trust in Sensors.
            initial_state (np.array, optional): Starting [x, y, z, vx, vy, vz]. 
                                                If None, starts at 0 with high uncertainty.
        """
        # 1. Time Step
        self.dt = dt

        # 2. State Vector (x) - [x, y, z, vx, vy, vz]
        if initial_state is not None:
            self.state = initial_state
        else:
            self.state = np.zeros((6, 1))

        # 3. State Transition Matrix (A) - The "Coast" Physics
        # x_new = x_old + v * dt
        self.A = np.eye(6)
        self.A[0, 3] = dt
        self.A[1, 4] = dt
        self.A[2, 5] = dt

        # 4. Control Input Matrix (B) - The "Thrust" Physics
        # Position += 0.5 * a * t^2
        # Velocity += a * t
        self.B = np.zeros((6, 3))
        self.B[0, 0] = 0.5 * dt**2
        self.B[1, 1] = 0.5 * dt**2
        self.B[2, 2] = 0.5 * dt**2
        self.B[3, 0] = dt
        self.B[4, 1] = dt
        self.B[5, 2] = dt

        # 5. Measurement Matrix (H)
        # We measure Position (first 3 rows), but ignore Velocity
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1  # Observe x
        self.H[1, 1] = 1  # Observe y
        self.H[2, 2] = 1  # Observe z

        # 6. Process Noise Covariance (Q)
        # Using the "Control Noise" method: Noise enters via acceleration
        self.process_variance = process_noise_std ** 2
        self.Q = self.B @ self.B.T * self.process_variance

        # 7. Measurement Noise Covariance (R) - Default values
        # This will be overridden dynamically during TERCOM updates
        std_x, std_y, std_z = meas_std_val
        self.R = np.diag([std_x**2, std_y**2, std_z**2])

        # 8. Error Covariance (P)
        # If we didn't provide an initial state, we are VERY uncertain (500)
        # If we did, we are reasonably confident (e.g., 10)
        if initial_state is None:
            self.P = np.eye(6) * 500.0
        else:
            self.P = np.eye(6) * 10.0

    def predict(self, accel_measurements):
        """
        Prediction Step: Uses Physics (INS) to estimate next state.
        accel_measurements: [ax, ay, az] from IMU
        """
        # u is the control input (acceleration)
        u = np.array(accel_measurements).reshape(3, 1)

        # x = A x + B u
        self.state = (self.A @ self.state) + (self.B @ u)

        # P = A P A^T + Q
        self.P = (self.A @ self.P @ self.A.T) + self.Q

    def update(self, measurement, R=None):
        """
        Correction Step: Uses Sensor (GPS/TERCOM) to correct estimate.
        measurement: [lat, lon, alt]
        R: Optional dynamic noise matrix (for TERCOM confidence)
        """
        z = np.array(measurement).reshape(3, 1)

        # Use dynamic R if provided (e.g. from TERCOM match score), else use default
        R_current = R if R is not None else self.R

        # 1. Kalman Gain: K = P H^T (H P H^T + R)^-1
        S = self.H @ self.P @ self.H.T + R_current
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 2. Update State: x = x + K (z - H x)
        y = z - (self.H @ self.state) # Innovation (Error)
        self.state = self.state + (K @ y)

        # 3. Update Uncertainty: P = (I - K H) P
        I = np.eye(6)
        self.P = (I - (K @ self.H)) @ self.P

    def set_process_noise(self, std_dev):
        """
        Call this when Flight Stage changes (e.g. BOOST -> CRUISE).
        BOOST = High std_dev (Trust GPS more)
        CRUISE = Low std_dev (Trust INS more)
        """
        self.process_variance = std_dev ** 2
        self.Q = self.B @ self.B.T * self.process_variance