"""Gyro attitude propagation with gravity correction; no simulated pose input."""
import numpy as np
from scipy.spatial.transform import Rotation


class Attitude:
    def __init__(self):
        self.rotation = None
        self.time = None

    def update(self, stamp, gyro, acceleration):
        gyro, acceleration = np.asarray(gyro), np.asarray(acceleration)
        if not np.isfinite(np.r_[stamp, gyro, acceleration]).all():
            raise ValueError('Nonfinite IMU measurement')
        norm = np.linalg.norm(acceleration)
        dt = stamp - self.time if self.time is not None else 0
        if self.rotation is None or dt <= 0 or dt > 0.1:
            if norm < 1:
                raise ValueError('Gravity initialization requires valid acceleration')
            x, y, z = acceleration
            self.rotation = Rotation.from_euler('xyz', [np.arctan2(y, z),
                                                       np.arctan2(-x, np.hypot(y, z)), 0])
        else:
            correction = np.zeros(3)
            if 8.0 < norm < 11.5:
                predicted = self.rotation.inv().apply([0, 0, 1])
                correction = 0.5 * np.cross(acceleration / norm, predicted)
            self.rotation = self.rotation * Rotation.from_rotvec((gyro + correction) * dt)
        self.time = stamp
        return self.rotation.as_quat()
