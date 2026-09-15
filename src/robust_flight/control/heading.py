class HeadingController:
    def __init__(self, kp):
        self.kp = kp

    def control(self, heading_error):
        return self.kp * heading_error
