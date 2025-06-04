import pybullet as p
import pybullet_data
import importlib.resources as resources

from .robot import Robot
from .lidar import Lidar
from .utils import RobotType, RobotColor
from .bullets import Bullet
from pynput import keyboard

# constants for robot movement
MAX_LINEAR_VELOCITY = 0.3
MAX_ANGULAR_VELOCITY = 2.0 

'''
Simulation manages our whole pybullet environment,
including its actors and the camera view. main_robot
is the controllable robot with the camera, robot_one
is the spinning robot.
'''
class Simulation():

    '''
    initializes the simulation with the robots
    '''
    def __init__(self, cam_hz=40, sim_speed=1):
        p.connect(p.GUI)
        # uncomment if you are running without OpenGL
        # p.connect(p.SHARED_MEMORY_SERVER)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)
        p.setPhysicsEngineParameter(enableFileCaching=1)
        p.setPhysicsEngineParameter(solverResidualThreshold=0.1)

        # load arena
        with resources.path('sim_node.models.arena', 'arena.urdf') as file_path:
            num=p.loadURDF(str(file_path), useFixedBase=True, basePosition=[0, 0, 0])
            print("arenaID:",num)

        # initialize our robots. The booleans determine if the robot is controllable
        self.main_robot = Robot(position=[4.2,2.8,0.5],
                                orientation=p.getQuaternionFromEuler([1.57,0,0]),
                                type=RobotType.INFANTRY,
                                color=RobotColor.BLUE)
        self.robot_one = Robot(position=[4.2,2.2,0.5],
                               orientation=p.getQuaternionFromEuler([0,0,0]),
                               type=RobotType.HERO,
                               color=RobotColor.RED)
        self.robot_two = Robot(position=[4.2,3.4,0.5],
                               orientation=p.getQuaternionFromEuler([1.57,0,0]),
                               type=RobotType.SENTRY,
                               color=RobotColor.BLUE)
        
        # which robot is controlled by keyboard inputs
        self.controlled_robot = self.main_robot

        # initialize the keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # other
        self.lidar = Lidar(self.main_robot.id, 0)
        self.step_amount = int(240 * sim_speed // cam_hz)

    '''
    handles key press events to set target velocities for the
    controllable robot based on keyboard input
    '''
    def on_press(self, key):
        if key == keyboard.Key.up:
            self.controlled_robot.target_linear_velocity[1] = -MAX_LINEAR_VELOCITY
        elif key == keyboard.Key.down:
            self.controlled_robot.target_linear_velocity[1] = MAX_LINEAR_VELOCITY
        elif key == keyboard.Key.left:
            self.controlled_robot.target_linear_velocity[0] = MAX_LINEAR_VELOCITY
        elif key == keyboard.Key.right:
            self.controlled_robot.target_linear_velocity[0] = -MAX_LINEAR_VELOCITY
        elif hasattr(key, 'char'):
            if key.char == 'a':
                self.controlled_robot.target_angular_velocity = MAX_ANGULAR_VELOCITY
            elif key.char == 'd':
                self.controlled_robot.target_angular_velocity = -MAX_ANGULAR_VELOCITY

    '''
    handles key release events to reset target velocities when 
    keys are released
    '''
    def on_release(self, key):
        if key in (keyboard.Key.up, keyboard.Key.down):
            self.controlled_robot.target_linear_velocity[1] = 0
        elif key in (keyboard.Key.left, keyboard.Key.right):
            self.controlled_robot.target_linear_velocity[0] = 0
        elif hasattr(key, 'char') and key.char in ('a', 'd'):
            self.controlled_robot.target_angular_velocity = 0
        #shoot bullet is set to 'q'
        elif hasattr(key, 'char'):
            if key.char == 'q':
                self.controlled_robot.fire_bullet()
        
    '''
    what the simulation should do every 1/CAM_HZ (defined in sim_node)
    physically step the simulation and update the velocities of the robots 
    '''
    def step(self, CAM_ENABLE):
        for _ in range(self.step_amount):
            p.stepSimulation()
        # update the bullets that are in the simulation
        Bullet.bullets = [bullet for bullet in Bullet.bullets if not bullet.update()]
        self.controlled_robot.update_movement()
        
        if (CAM_ENABLE): return self.main_robot.get_camera()

    def get_scan(self):
        return self.lidar.scan()

    def __del__(self):
        p.disconnect()