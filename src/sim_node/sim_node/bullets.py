import pybullet as p
import numpy as np

import importlib.resources as resources

class Bullet:
    bullets = []

    '''
    creates a bullet every time the robot shoots
    @param position: position of the robot
    @param direction: direction of the robot
    @param speed: speed of the bullet
    '''
    def __init__(self, position, direction, speed=20.0):
        forward_direction = np.array(p.getMatrixFromQuaternion(direction)).reshape(3, 3) @ np.array([0, 0, 1])
        self.direction = forward_direction / np.linalg.norm(direction)
        #bullet position is slightly in front of the robot, and slightly above the ground
        self.speed = speed
        self.position = np.array(position)
        self.velocity = np.array(self.direction * self.speed)
        
        #for the timer to remove the bullet
        self.update_count = 0

        with resources.path('sim_node.models.bullet', 'bullet.urdf') as file_path:
            self.id = p.loadURDF(str(file_path), self.position, [0, 0, 0, 1])
        p.resetBaseVelocity(self.id, self.velocity, 
            angularVelocity=[0, 0, 0])
        
        self.bullets.append(self)

    '''
    updates the bullet's position and checks for collisions
    @return True if the bullet has been removed
    '''
    def update(self):
        self.check_collision()
        self.update_count += 1
        if self.update_count > 40:
            p.removeBody(self.id)
            return True
        return False

    '''
    checks if the bullet hit something
    @return: the id of the object hit (-1 if nothing)
    '''
    def check_collision(self):
        ray_start = self.position
        ray_end = self.position + self.velocity * 0.1

        ray_test = p.rayTest(ray_start, ray_end)
        return ray_test[0][0]
    