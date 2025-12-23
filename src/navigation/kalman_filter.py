import numpy as np
import time

class KalmanFilter:
    def __init__(self, dt, ux, uy, uz, std_noise, std_mea):

        # Sampling time
        self.dt = dt

        # Control input variable matrix
        self.u = np.matrix([ux, uy, uz])
        
        # State transition matrix (A) 6 * 6 for 3D
        self.A = np.eye(6) # creating identity matrix, 0 and 1s are all in place already
        self.A[0, 3] = self.dt
        self.A[1, 4] = self.dt
        self.A[2, 5] = self.dt

        # 
        self.B = np.eye()
        


        