class FromSTM32:
    def __init__(self, x_vel=0.0, y_vel=0.0, pitch=0.0, yaw=0.0, pitch_vel=0.0, yaw_vel=0.0):
        self.x_vel = x_vel
        self.y_vel = y_vel
        self.pitch = pitch
        self.yaw = yaw
        self.pitch_vel = pitch_vel
        self.yaw_vel = yaw_vel

class RobotType:
    INFANTRY = "infantry"
    HERO = "hero"
    SENTRY = "sentry"

class RobotColor:
    RED = "red"
    BLUE = "blue"