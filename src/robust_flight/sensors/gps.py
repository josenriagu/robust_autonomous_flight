import numpy as np


class GPS:
    """Model a GPS receiver that reports position and velocity.

    Position comes from timing the satellite signals, while velocity comes
    from their Doppler shift, so velocity is far less noisy than position.
    Samples arrive at a fixed rate, and each sample can be lost.
    """

    def __init__(
        self,
        rng,
        rate=5.0,
        position_noise=1.0,
        velocity_noise=0.1,
        position_bias=(0.0, 0.0),
        dropout_probability=0.0,
    ):
        self.rng = rng
        self.period = 1.0 / rate
        self.position_noise = position_noise
        self.velocity_noise = velocity_noise
        self.position_bias = np.array(position_bias)
        self.dropout_probability = dropout_probability
        self.next_sample_time = self.period

    def measure(self, time, state):
        # a small tolerance absorbs floating-point error in the accumulated time
        if time + 1e-9 < self.next_sample_time:
            return None
        self.next_sample_time += self.period

        # every sample draws the same random numbers whether or not it is
        # lost, so changing the dropout probability leaves the noise unchanged
        position_error = self.rng.normal(0.0, self.position_noise, 2)
        velocity_error = self.rng.normal(0.0, self.velocity_noise, 2)
        lost = self.rng.random() < self.dropout_probability
        if lost:
            return None

        px, py, v, psi = state
        position = np.array([px, py]) + self.position_bias + position_error
        velocity = np.array([v * np.cos(psi), v * np.sin(psi)]) + velocity_error
        return np.concatenate([position, velocity])
