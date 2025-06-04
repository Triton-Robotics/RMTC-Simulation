import numpy as np
import pybullet as p

'''
models a 2d LIDAR mounted on sentry, capable of performing
a 2d laser scan and returning distance measurements
'''
class Lidar:
    '''
    initializes the LIDAR object with the given parameters
    '''
    def __init__(self, robot_id, mount_link_id, num_rays=1450, max_range=12.0, noise=0.035, min_range=0.15, fov=260, position=(0,0,0.2)):
        self.robot_id = robot_id
        self.mount_link_id = mount_link_id
        self.num_rays = num_rays
        self.max_range = max_range
        self.min_range = min_range
        self.fov = fov
        self.noise = noise
        self.position = position

    '''
    calculates the LIDAR's world position and orientation
    based on turret mount
    '''
    def calculate_pos_or(self):
        mount_link_state = p.getLinkState(self.robot_id, self.mount_link_id)
        parent_pos, parent_or = mount_link_state[0], mount_link_state[1]

        rot_matrix = np.array(p.getMatrixFromQuaternion(parent_or)).reshape(3, 3)
        rotated_offset = np.dot(rot_matrix, self.position)

        lidar_pos = np.add(parent_pos, rotated_offset)
        lidar_or = parent_or

        return lidar_pos, lidar_or

    '''
    generates the start and end points of rays for LIDAR scan
    '''
    def generate_rays(self, lidar_pos, lidar_or):
        rays_from = []
        rays_to = []

        ray_angles = np.linspace(0, 2*np.pi, self.num_rays)

        lidar_rotation_matrix = np.array(p.getMatrixFromQuaternion(lidar_or)).reshape(3, 3)

        for angle in ray_angles:
            local_ray = np.array([
                self.max_range * np.cos(angle),
                0,
                self.max_range * np.sin(angle)
            ])
            world_ray = np.dot(lidar_rotation_matrix, local_ray)
            rays_from.append(lidar_pos)
            rays_to.append(np.add(lidar_pos, world_ray))
        
        return rays_from, rays_to

    '''
    filters and returns the distances array based on LIDAR's fov
    '''
    def filter_distances(self, distances):
        half_rays = self.num_rays/2
        fov_half_rays = self.fov * self.num_rays/360

        for i in range(self.num_rays):
            if (i > half_rays + (fov_half_rays - half_rays)/2 
                and i < self.num_rays - (fov_half_rays - half_rays)/2):
                distances[i] = float('inf')

        return distances

    '''
    performs a LIDAR scan and returns the distances
    '''
    def scan(self):
        lidar_pos, lidar_or = self.calculate_pos_or()
        rays_from, rays_to = self.generate_rays(lidar_pos, lidar_or)

        results = p.rayTestBatch(rays_from, rays_to)

        distances = []
        for result in results:
            hit_fraction = result[2]

            distance = hit_fraction * self.max_range
            if distance < self.min_range or distance > self.max_range:
                distance = float('inf')
            else:
                distance = distance + np.random.normal(0, self.noise * distance/self.max_range)

            distances.append(distance)

        return self.filter_distances(distances)


        
