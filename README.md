# Simulation
Pybullet simulation of our robots. Should listen to server and movement requests in order to simulate how the robot would move in real-time. 

# Building
Before you begin, make sure to install dependencies with `rosdep install -i --from-path src --rosdistro humble -y`. Then, this package is dependent on tr_messages from [TR-Messages-24](https://github.com/Triton-Robotics/TR-Messages-24.git), so make sure to place it in the same workspace directory. From there, you simply have to make sure  to build tr_messages then sim_node.

```
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select tr_messages
colcon build --packages-select sim_node
```

# Running
Once the packages are built, you can startup the simulation with.
```
source install/setup.bash
ros2 run sim_node sim_node
```
The robot will then be loaded and start a server that listens to /get_data and /write_data. To try calling commands and testing the robot, you can do so using rqt or running these commands in the terminal:
```
source install/setup.bash
ros2 service call /write_data tr_messages/srv/WriteSerial "{pitch: 0.5, yaw: 1.2, shoot: false}"
ros2 service call /get_data tr_messages/srv/ListenSerial
```

# Parameters
Sim_node has multiple parameters to allow for configurability of the system at runtime. To run with parameters, you can use `params.yaml` from the [main repo](https://github.com/Triton-Robotics/TR-Autonomy-2024-2025.git). 
```
source install/setup.bash
ros2 run sim_node sim_node --ros-args --params-file params.yaml
```
Here is a quick list and description of each parameter:
- **camera_resolution:** A scalar multiplied to the default resolution of 1280x1024
- **cam_hz:** The frequency which the camera will publish images (FPS of cam)
- **fov:** The vertical FOV used by the camera
- **lidar_enable:** Whether or not to simulate using a LIDAR
- **lidar_hz:** The frequency at which LIDAR will publish scans
- **sim_speed:** A scalar that determines the simulation speed relative to real time

# Using Without OpenGL
Some machines can't run OpenGL (at least Macs with an M1 chip seem to have this problem), so we've created an alternative with two additional nodes. 

Before building any code, you'll want to go into `src/sim_node/sim_node/simulation.py` and change `p.connect(p.GUI)` into `p.connect(p.SHARED_MEMORY_SERVER)` in the `__init__` method. This stops the simulation from trying to create an OpenGL window, allowing you to run it without OpenGL.

Then when you build 'tr_messages' and 'sim_node', also run these two commands:
```
colcon build --packages-select freecam
colcon build --packages-select viewer_node
```

Then after starting the simulation (as in the "Running" section), you can run these commands in two new terminal windows:
```
source install/setup.bash
ros2 run freecam freecam
```
```
source install/setup.bash
ros2 run viewer_node viewer_node
```

This should pop up a new window. If you press G while focused on that window, an overview image of the simulation should appear. Then you can use W, A, S, D to move around laterally, F and SPACE to move vertically, and click + dragging your mouse to move the direction of the camera.

# LIDAR and Mapping
To visualize LIDAR data:
```
ros2 launch src/view_lidar.launch.py
```

Use arrow keys to move the robot around, keys A and D to rotate the robot left and right

# Mapping
Launch Cartographer SLAM:
```
ros2 launch src/launch/slam/cartographer_map.py
```

RViz2 should automatically start and display the map being built.
