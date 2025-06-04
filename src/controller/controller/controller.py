import rclpy
from rclpy.node import Node
from messages.msg import Control
from pynput import keyboard

'''
Controller handles keyboard input to control the robot's velocities
and actions. It publishes the control messages to the 'control' topic.
'''
class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.control_pub = self.create_publisher(Control, 'control', 10)
        self.pub_timer = self.create_timer(0.1, self.publish_control)

        # initialize parameters
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('max_pitch_velocity', 1.0)
        self.declare_parameter('max_yaw_velocity', 1.0)

        # get parameters
        self.MAX_LINEAR_VELOCITY = self.get_parameter('max_linear_velocity').value
        self.MAX_ANGULAR_VELOCITY = self.get_parameter('max_angular_velocity').value
        self.MAX_PITCH_VELOCITY = self.get_parameter('max_pitch_velocity').value
        self.MAX_YAW_VELOCITY = self.get_parameter('max_yaw_velocity').value

        self.x_vel = 0.0
        self.y_vel = 0.0
        self.angular_vel = 0.0
        self.pitch_vel = 0.0
        self.yaw_vel = 0.0
        self.shoot = False

        # initialize the keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    '''
    publishes the current target velocities of the controlled robot
    '''
    def publish_control(self):
        control_msg = Control()
        control_msg.x_vel = self.x_vel
        control_msg.y_vel = self.y_vel
        control_msg.angular_vel = self.angular_vel
        control_msg.pitch_vel = self.pitch_vel
        control_msg.yaw_vel = self.yaw_vel
        control_msg.shoot = self.shoot
        self.control_pub.publish(control_msg)

    '''
    handles key press events to set target velocities for the
    controllable robot based on keyboard input
    '''
    def on_press(self, key):
        if key == keyboard.Key.up:
            self.x_vel = self.MAX_LINEAR_VELOCITY
        elif key == keyboard.Key.down:
            self.x_vel = -self.MAX_LINEAR_VELOCITY
        elif key == keyboard.Key.left:
            self.y_vel = -self.MAX_LINEAR_VELOCITY
        elif key == keyboard.Key.right:
            self.y_vel = self.MAX_LINEAR_VELOCITY
        elif hasattr(key, 'char'):
            if key.char == 'a':
                self.angular_vel = self.MAX_ANGULAR_VELOCITY
            elif key.char == 'd':
                self.angular_vel = -self.MAX_ANGULAR_VELOCITY
            elif key.char == 'h':
                self.pitch_vel = self.MAX_PITCH_VELOCITY
            elif key.char == 'j':
                self.pitch_vel = -self.MAX_PITCH_VELOCITY
            elif key.char == 'k':
                self.yaw_vel = self.MAX_YAW_VELOCITY
            elif key.char == 'l':
                self.yaw_vel = -self.MAX_YAW_VELOCITY
        elif key == keyboard.Key.space:
            self.shoot = True

    '''
    handles key release events to reset target velocities when 
    keys are released
    '''
    def on_release(self, key):
        if key in (keyboard.Key.up, keyboard.Key.down):
            self.x_vel = 0.0
        elif key in (keyboard.Key.left, keyboard.Key.right):
            self.y_vel = 0.0
        elif hasattr(key, 'char'):
            if key.char in ('a', 'd'):
                self.angular_vel = 0.0
            elif key.char in ('h', 'j'):
                self.pitch_vel = 0.0
            elif key.char in ('k', 'l'):
                self.yaw_vel = 0.0
        elif key == keyboard.Key.space:
            self.shoot = False

'''
main function for our simulation node
initialize the robots and the simulation node,
then runs the simulation in a loop
'''
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()