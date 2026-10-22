import numpy as np


class IMU:
    """Model an IMU that measures acceleration and yaw rate.

    In the planar model these are exactly the applied control inputs, so the
    IMU measures the applied control with white noise and a constant bias on
    each channel. It samples every simulation step. The noise is given per
    sample, so its effect depends on the time step.
    """

    def __init__(
        self,
        rng,
        acceleration_noise=0.05,
        yaw_rate_noise=0.01,
        acceleration_bias=0.0,
        yaw_rate_bias=0.0,
    ):
        self.rng = rng
        self.noise = np.array([acceleration_noise, yaw_rate_noise])
        self.bias = np.array([acceleration_bias, yaw_rate_bias])

    def measure(self, applied_control):
        error = self.rng.normal(0.0, 1.0, 2) * self.noise
        return np.array(applied_control) + self.bias + error
