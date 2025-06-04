import math
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from .simulation import Simulation
import time

'''
sim_node handles exposing the simulation as a ROS2 service node. It provides
a service to write data to the simulation and a service to listen to the
data from the simulation.
'''
class Sim_Node(Node):
    '''
    initializes the simulation node, its parameters, its services,
    and its publishers
    '''
    def __init__(self):
        super().__init__('sim_node')
        # initialize parameters
        self.declare_parameter('sim_speed', 1.0)
        self.declare_parameter('cam_hz', 20.0)
        self.declare_parameter('lidar_hz', 5.5)
        self.declare_parameter('lidar_enable', True)
        self.declare_parameter('cam_enable', True)
        self.declare_parameter('camera_resolution', 1.0)
        self.declare_parameter('fov', 27.95)

        # get parameters
        SIM_SPEED = self.get_parameter('sim_speed').value
        CAM_HZ = self.get_parameter('cam_hz').value
        LIDAR_HZ = self.get_parameter('lidar_hz').value
        LIDAR_ENABLE = self.get_parameter('lidar_enable').value
        CAM_ENABLE = self.get_parameter('cam_enable').value
        CAMERA_RESOLUTION = self.get_parameter('camera_resolution').value
        FOV = self.get_parameter('fov').value

        # initializes services and publishers
        self.bridge = CvBridge()
        self.simulation = Simulation(cam_hz=CAM_HZ, sim_speed=SIM_SPEED)
        self.step_simulation = self.create_timer(1 / CAM_HZ, self.step_simulation)
        self.simulation.main_robot.set_camera(fov=FOV, camera_resolution=CAMERA_RESOLUTION)
        self.image_pub = self.create_publisher(Image, 'camera/image', 10)
        if LIDAR_ENABLE:
            self.get_lidar = self.create_timer(1 / LIDAR_HZ, self.get_lidar)
            self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
    
        self.CAM_ENABLE = CAM_ENABLE

    '''
    steps the simulation and publishes the rendered image
    '''
    def step_simulation(self):
        if (self.CAM_ENABLE):
            rendered_image = self.simulation.step(self.CAM_ENABLE)
            image = self.bridge.cv2_to_imgmsg(rendered_image, encoding='bgr8')
            self.image_pub.publish(image)
        else:
            self.simulation.step(self.CAM_ENABLE)
    
    '''
    gets the LIDAR scan and publishes it
    '''
    def get_lidar(self):
        distances = self.simulation.get_scan()
        self.scan_pub.publish(get_scan_msg(distances, time.time()))

'''
generates LaserScan message based on provided 
distances and current time
'''
def get_scan_msg(distances, current_time):
    scan = LaserScan()
    scan.header.stamp.sec = int(current_time)
    scan.header.stamp.nanosec = int((current_time - int(current_time)) * 1e9)
    scan.header.frame_id = 'laser'
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = (2 * math.pi) / len(distances)
    scan.time_increment = 0.0
    scan.scan_time = 1./5.5 
    scan.range_min = 0.5
    scan.range_max = 12.0
    scan.ranges = distances
    scan.intensities = [40. for _ in range(len(distances))]
    return scan


'''
main function for our simulation node
initialize the robots and the simulation node,
then runs the simulation in a loop
'''
def main(args=None):
    rclpy.init(args=args)
    sim_node = Sim_Node()
    rclpy.spin(sim_node)
    sim_node.destroy_node()
    rclpy.shutdown()