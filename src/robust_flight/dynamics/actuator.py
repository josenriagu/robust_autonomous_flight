import numpy as np

class Actuator:
    """Model actuator limits for the aircraft control inputs.
    The current model assumes symmetric turn-rate authority, meaning
    the aircraft has the same maximum turn rate magnitude to the left
    and right. This can be generalized later if asymmetric actuator
    behavior needs to be modeled.
    """
    def __init__(
        self,
        min_acceleration,
        max_acceleration,
        max_turn_rate
    ):
        self.min_acceleration = min_acceleration
        self.max_acceleration = max_acceleration
        self.max_turn_rate = max_turn_rate

    def apply(self, control):
        acceleration, turn_rate = control
        acceleration = np.clip(
            acceleration,
            self.min_acceleration,
            self.max_acceleration
        )
        turn_rate = np.clip(
            turn_rate,
            -self.max_turn_rate,
            self.max_turn_rate
        )
        return np.array([acceleration, turn_rate])

actuator = Actuator(
    min_acceleration=-2.0,
    max_acceleration=3.0,
    max_turn_rate=np.pi/3
)

actuator.apply(np.array([10.0, 2.0]))
actuator.apply(np.array([-10.0, -2.0]))