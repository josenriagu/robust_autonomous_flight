from collections import deque

import numpy as np

# a filter told a sensor is perfect divides by zero, so every assumed noise
# level is at least this small value
MIN_NOISE = 1e-6


class EKF:
    """Estimate the aircraft state [px, py, v, psi] from IMU and GPS.

    Every step predicts the state with the same planar model the aircraft
    follows, driven by the measured acceleration and yaw rate. Each GPS
    sample of position and velocity then corrects the prediction. The
    filter assumes the noise levels it is given are the sensors' true ones.
    """

    def __init__(
        self,
        initial_state,
        initial_covariance,
        acceleration_noise,
        yaw_rate_noise,
        position_noise,
        velocity_noise,
    ):
        self.state = np.array(initial_state, dtype=float)
        self.covariance = np.array(initial_covariance, dtype=float)

        acceleration_noise, yaw_rate_noise, position_noise, velocity_noise = (
            max(noise, MIN_NOISE)
            for noise in (
                acceleration_noise,
                yaw_rate_noise,
                position_noise,
                velocity_noise,
            )
        )
        self.imu_covariance = np.diag([acceleration_noise**2, yaw_rate_noise**2])
        self.gps_covariance = np.diag(
            [position_noise**2, position_noise**2, velocity_noise**2, velocity_noise**2]
        )

    def predict(self, imu_measurement, dt):
        px, py, v, psi = self.state
        acceleration, yaw_rate = imu_measurement
        cos_psi, sin_psi = np.cos(psi), np.sin(psi)

        self.state = self.state + dt * np.array(
            [v * cos_psi, v * sin_psi, acceleration, yaw_rate]
        )

        # Jacobian of one step of the model with respect to the state
        F = np.eye(4)
        F[0, 2] = dt * cos_psi
        F[0, 3] = -dt * v * sin_psi
        F[1, 2] = dt * sin_psi
        F[1, 3] = dt * v * cos_psi

        # IMU noise enters speed and heading through one time step
        G = np.zeros((4, 2))
        G[2, 0] = dt
        G[3, 1] = dt

        self.covariance = F @ self.covariance @ F.T + G @ self.imu_covariance @ G.T

    def update(self, gps_measurement):
        px, py, v, psi = self.state
        cos_psi, sin_psi = np.cos(psi), np.sin(psi)

        # GPS measures position and the velocity vector
        predicted = np.array([px, py, v * cos_psi, v * sin_psi])
        H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, cos_psi, -v * sin_psi],
                [0.0, 0.0, sin_psi, v * cos_psi],
            ]
        )

        innovation = np.array(gps_measurement) - predicted
        innovation_covariance = H @ self.covariance @ H.T + self.gps_covariance
        gain = np.linalg.solve(innovation_covariance, H @ self.covariance).T

        self.state = self.state + gain @ innovation

        # the Joseph form keeps the covariance symmetric and positive definite
        correction = np.eye(4) - gain @ H
        self.covariance = (
            correction @ self.covariance @ correction.T
            + gain @ self.gps_covariance @ gain.T
        )


class DelayCompensatedEKF:
    """Apply each GPS fix at the moment it was taken, however late it arrives.

    The filter remembers its recent states, covariances and IMU readings. A
    late fix rewinds the filter to the step the fix was taken, corrects it
    there, then replays the remembered IMU readings to bring the estimate
    back to the present. With no delay this is identical to the plain EKF.
    """

    def __init__(self, ekf, history_length):
        self.ekf = ekf
        self.history = deque(maxlen=history_length)

    @property
    def state(self):
        return self.ekf.state

    @property
    def covariance(self):
        return self.ekf.covariance

    def predict(self, imu_measurement, dt, time):
        self.ekf.predict(imu_measurement, dt)
        self.history.append(
            [
                time,
                imu_measurement,
                dt,
                self.ekf.state.copy(),
                self.ekf.covariance.copy(),
            ]
        )

    def update(self, gps_measurement, measured_time):
        # find the remembered step the fix was taken at; a fix older than the
        # history cannot be placed and is not applied
        steps = [
            i
            for i, entry in enumerate(self.history)
            if abs(entry[0] - measured_time) < 1e-6
        ]
        if not steps:
            return False

        # rewind, correct at the time of the fix, then replay to the present
        start = steps[0]
        self.ekf.state = self.history[start][3].copy()
        self.ekf.covariance = self.history[start][4].copy()
        self.ekf.update(gps_measurement)
        self.history[start][3] = self.ekf.state.copy()
        self.history[start][4] = self.ekf.covariance.copy()

        for entry in list(self.history)[start + 1 :]:
            self.ekf.predict(entry[1], entry[2])
            entry[3] = self.ekf.state.copy()
            entry[4] = self.ekf.covariance.copy()
        return True
