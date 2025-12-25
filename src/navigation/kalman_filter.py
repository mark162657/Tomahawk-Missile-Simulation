import numpy as np
import time

class KalmanFilter:
    def __init__(self, dt, ux, uy, uz, process_noise_std, std_mea):

        # Sampling time
        self.dt = dt

        # Measurement error (GPS/TERCOM), definetly a simplfy of real life case
        self.std_mea = std_mea

        # Control input variable matrix
        self.u = np.matrix([ux, uy, uz])
        
        # State transition matrix (A) 6 * 6 for 3D
        self.A = np.eye(6) # creating identity matrix, 0 and 1s are all in place already
        self.A[0, 3] = self.dt
        self.A[1, 4] = self.dt
        self.A[2, 5] = self.dt

        # Control input matrix (B) 6 * 3 for 3D
        self.B = np.zeros((6, 3))
        
        self.B[0, 0] = 0.5 * self.dt ** 2
        self.B[1, 1] = 0.5 * self.dt ** 2
        self.B[2, 2] = 0.5 * self.dt ** 2

        self.B[3, 0] = self.dt
        self.B[4, 1] = self.dt
        self.B[5, 2] = self.dt

        # Measurement matrix (H) 
        # maps the 6 variable internal state to the 3 variable sensor measurement space, selecting only positional components, ignore velocities
        self.H = zeros((3, 6))
        self.H[0, 0] = 1  # Observe x
        self.H[1, 1] = 1  # Observe y
        self.H[2, 2] = 1  # Observe z

        # Process noise covariance matrix (Q)
        # internal uncertainty: how weather / physic disturb the missile
        self.Q = self.Q = (self.B @ self.B.T) * (process_noise_std ** 2) # projecting acceleration noise through the physical control paths

        # Sensor noise covariance matrix (R)
        # scenario 1: GPS (+/- 2.5m. Vertical is usually 1.5x worse.)
        self.R_GPS = np.diag([2.5 ** 2, 4.0 ** 2, 2.5 ** 2])

        # scenario 2: TERCOM (12m deviation)
        # lateral Accuracy: +/- 10-15m (Grid dependent)
        # vertical Accuracy: Radar Altimeter is very precise (+/- 1m)
        self.R_TERCOM = np.diag([12.0 ** 2, 4.0 ** 2, 2.5 ** 2])

        # scenario 3: default to GPS (most cases for accuracy, TERCOM serve as a helper)
        self.R = self.R_GPS

        # Error covariance matrix
        # we assume we are not very certain where we are (~50m initial error)
        self.P = np.eye(6) * 100 # * 100 as we assume 100m drift, can be adjusted accordingly

    
    def predict(self, acc_input):
        u = np.array(acc_input)

        # Predictive state x = Ax + Bu:
        self.x = (self.A @ self.x) + (self.B @ u)
        
        # Covariance prediction P = AP * A.T + Q: 
        self.P = (self.A @ self.P @ self.A.T) + self.Q

    def update(self, measurement, sensor_type="GPS": str):
        # Set the R matrix to different value based on sensor_type
        if sensor_type = "TERCOM":
            R_current = self.R_TERCOM
        else:
            R_current = self.R_GPS


        # Calculate
 
        # Kalman gain (KG)
        KG = self.P @ self.H.T @ np.linalg.inv((H @ self.P @ H.T) + R_current)
        



        