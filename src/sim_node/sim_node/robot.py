import pybullet as p
import importlib.resources as resources
import numpy as np
from cv2 import cvtColor, COLOR_BGR2RGB
from .bullets import Bullet

from .utils import FromSTM32, RobotColor, RobotType

# constants obtained from our camera we multiply by camera_resolution
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 1024
ASPECT_RATIO = CAMERA_WIDTH / CAMERA_HEIGHT

# constants for robot movement
MAX_LINEAR_VELOCITY = 2.5
MAX_ANGULAR_VELOCITY = 5.0 

'''
robots.py is a file for the robot class, where each
robot is a urdf mesh of infantry
'''
class Robot():
    '''
    load a robot into the pybullet simulation at environment
    @param position: 3D [x,y,z] position of the robot
    @param orientation: Quaternion orientation of the robot
    @param color: the lightbar color of the robot (red or blue)
    '''
    def __init__(self, position, orientation, controllable=False, type=RobotType.SENTRY, color=RobotColor.RED):
        urdf = f"{type}-{color}.urdf"
        with resources.path(f'sim_node.models.{type}', urdf) as file_path:
            self.id = p.loadURDF(str(file_path), position, orientation)
        self.set_camera()
        self.shoot = False
        
        self.current_linear_velocity = np.zeros(3)
        self.current_angular_velocity = 0.0
        self.target_linear_velocity = np.zeros(3)
        self.target_angular_velocity = 0.0

        self.cam_coords = np.zeros(3)
        self.cam_orientation = np.zeros(4)

    def set_camera(self, fov=27.95, camera_resolution=1.0):
        self.fov = fov
        self.width = int(CAMERA_WIDTH * camera_resolution)
        self.height = int(CAMERA_HEIGHT * camera_resolution)

    '''
    returns the state of the robot
    pitch and yaw motor angles are inverted 
    to match the standard coordinate system
    @return: FromSTM32 state object
    '''
    def get_state(self):
        yaw_state = p.getJointState(bodyUniqueId=self.id, jointIndex=0)
        pitch_state = p.getJointState(bodyUniqueId=self.id, jointIndex=1)
        body_vel = p.getBaseVelocity(bodyUniqueId=self.id)
        data = FromSTM32(x_vel=body_vel[0][0],
                         y_vel=body_vel[0][2],
                         pitch=-pitch_state[0],
                         pitch_vel=pitch_state[1],
                         yaw=-yaw_state[0],
                         yaw_vel=yaw_state[1]
                        )
        return data

    '''
    moves the turret by the pitch and yaw params
    @param pitch: float in radians to move turret up and down (up is positive)
    @param yaw: float radians to move turret side to side (right is positive)
    '''
    def set_turret(self, pitch=0.0, yaw=0.0, shoot=False):
        # yaw is jointIdx 0, pitch is jointIdx 1
        p.setJointMotorControlArray(bodyUniqueId=self.id,
                                    jointIndices=[0, 1],
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=[-yaw, -pitch],
                                    targetVelocities=[0.2, 0.2])
        self.shoot = shoot
    
    '''
    calculates the camera view from the given robot's turret
    @return what main robot "sees", list of [R, G, B, A]
    '''
    def get_camera(self):
        self.cam_coords = np.array(p.getLinkState(self.id, 1)[0])
        self.cam_orientation = np.array(p.getLinkState(self.id, 1)[1])
        # use our orientation and position to calculate our forward direction
        cam_direction = np.array(p.getMatrixFromQuaternion(self.cam_orientation)).reshape(3, 3) @ np.array([0, 0, 1]) + self.cam_coords

        # calculate our camera view
        forward_direction = cam_direction
        view_mat = p.computeViewMatrix(cameraEyePosition=self.cam_coords, cameraTargetPosition=forward_direction, cameraUpVector=[0, 0, 1])
        proj_mat = p.computeProjectionMatrixFOV(self.fov, ASPECT_RATIO, .1, 100)
        _, _, image, _, _ = p.getCameraImage(width=self.width, height=self.height, viewMatrix=view_mat, projectionMatrix=proj_mat, renderer=p.ER_TINY_RENDERER)
        image = image[:, :, :3]
        return cvtColor(image, COLOR_BGR2RGB)

    '''
    updates the current velocities and applies them to the robot
    '''
    def update_movement(self):
        for i in range(2):
            if self.current_linear_velocity[i] < self.target_linear_velocity[i]:
                self.current_linear_velocity[i] = min(self.current_linear_velocity[i] + 
                                                      MAX_LINEAR_VELOCITY/3, self.target_linear_velocity[i])
            elif self.current_linear_velocity[i] > self.target_linear_velocity[i]:
                self.current_linear_velocity[i] = max(self.current_linear_velocity[i] - 
                                                      MAX_LINEAR_VELOCITY/3, self.target_linear_velocity[i])

        if self.current_angular_velocity < self.target_angular_velocity:
            self.current_angular_velocity = min(self.current_angular_velocity + 
                                                MAX_ANGULAR_VELOCITY/3, self.target_angular_velocity)
        elif self.current_angular_velocity > self.target_angular_velocity:
            self.current_angular_velocity = max(self.current_angular_velocity - 
                                                MAX_ANGULAR_VELOCITY/3, self.target_angular_velocity)

        _, orientation = p.getBasePositionAndOrientation(self.id)
        _, _, yaw = p.getEulerFromQuaternion(orientation)

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        rotation_matrix_2d = np.array([[cos_yaw, -sin_yaw],
                                       [sin_yaw, cos_yaw]])

        local_velocity_2d = np.array([self.current_linear_velocity[0], self.current_linear_velocity[1]])
        global_velocity_2d = np.dot(rotation_matrix_2d, local_velocity_2d)

        global_velocity = np.array([global_velocity_2d[0], global_velocity_2d[1], 0])

        p.resetBaseVelocity(self.id, 
            linearVelocity=global_velocity.tolist(), 
            angularVelocity=[0, 0, self.current_angular_velocity])
        
        # if the robot is shooting, fire a bullet
        if self.shoot:
            self.fire_bullet()
    
    '''
    fires a bullet from the robot
    '''
    def fire_bullet(self):
        Bullet(self.cam_coords, self.cam_orientation)
    
    def __del__(self):
        p.disconnect()
        