class SpeedController:
    def __init__(self, kp):
        self.kp = kp

    def control(self, speed_error):
        return self.kp * speed_error
