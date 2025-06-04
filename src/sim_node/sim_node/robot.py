import pybullet as p
import importlib.resources as resources
import numpy as np
from cv2 import cvtColor, COLOR_BGR2RGB
from .bullets import Bullet

from .utils import RobotColor, RobotType

# constants obtained from our camera we multiply by camera_resolution
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 1024
ASPECT_RATIO = CAMERA_WIDTH / CAMERA_HEIGHT

'''
robots.py is a file for the robot class, where each robot is a pybullet URDF model.
The robot class contains methods to control the robot's turret and get the camera 
view from the robot's turret.
'''
class Robot():
    '''
    load a robot into the pybullet simulation at environment
    @param position: 3D [x,y,z] position of the robot
    @param orientation: Quaternion orientation of the robot
    @param color: the lightbar color of the robot (red or blue)
    '''
    def __init__(self, position, orientation, type=RobotType.SENTRY, color=RobotColor.RED):
        urdf = f"{type}-{color}.urdf"
        with resources.path(f'sim_node.models.{type}', urdf) as file_path:
            self.id = p.loadURDF(str(file_path), position, orientation)
        self.set_camera()
        self.shoot = False
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.angular_vel = 0.0

        self.cam_coords = np.zeros(3)
        self.cam_orientation = np.zeros(4)

    def set_camera(self, fov=27.95, camera_resolution=1.0):
        self.fov = fov
        self.width = int(CAMERA_WIDTH * camera_resolution)
        self.height = int(CAMERA_HEIGHT * camera_resolution)
    
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
    shoots a bullet if the shoot command is given
    '''
    def shoot_bullet(self):
        if self.shoot:
            Bullet(self.cam_coords, self.cam_orientation)
    
    '''
    moves the turret by the pitch and yaw params
    @param pitch_vel moves the turret up and down (up is positive)
    @param yaw_vel moves the turret side to side (counter-clockwise is positive)
    '''
    def move_turret(self, pitch_vel, yaw_vel):
        # yaw is jointIdx 0, pitch is jointIdx 1
        p.setJointMotorControlArray(bodyUniqueId=self.id,
                                    jointIndices=[0, 1],
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocities=[yaw_vel, pitch_vel])

    '''
    updates the current velocities and applies them to the robot
    x_vel is forward/backward velocity, positive is forward
    y_vel is left/right velocity, positive is right
     is the angular velocity, positive is counter-clockwise
    '''
    def drive(self):
        _, orientation = p.getBasePositionAndOrientation(self.id)
        _, _, yaw = p.getEulerFromQuaternion(orientation)

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        rotation_matrix_2d = np.array([[cos_yaw, -sin_yaw],
                                       [sin_yaw, cos_yaw]])

        local_velocity_2d = np.array([self.y_vel, self.x_vel])
        global_velocity_2d = np.dot(rotation_matrix_2d, local_velocity_2d)

        global_velocity = np.array([global_velocity_2d[0], global_velocity_2d[1], 0])

        p.resetBaseVelocity(self.id, 
            linearVelocity=global_velocity.tolist(), 
            angularVelocity=[0, 0, self.angular_vel])
    
    def __del__(self):
        p.disconnect()
        