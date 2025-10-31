# Crazyflie Simulation Environment

This code contains the simulation stack for crazyflies. It is a modified version of the codebase used for the ICUAS25 and ICUAS26 Competitions. Original versions can be found [here](https://github.com/larics/icuas25_competition) and [here](https://github.com/larics/icuas26_competition).

## Task 1: Image processing

Here you task is to subscribe to an image, process it and publish to a new topic. A sample script is given to you in `mini_hero_pkg/scripts/process_image.py` file, where you need to implement your logic to draw something on image first, then try aruco detection on the image.

### Step 1: Remove your old container

```bash
docker rm crazyflies_sim_cont
```

### Step 2: Pull the latest changes from git

```bash
cd /path/to/folder/undergrad_project_larics_2025
git pull
```

### Step 3: Start the new container

Navigate to `crazyflies/crazyflies_sim/startup/first_run.sh`. Replace `student_ws` by `mini_hero_pkg`. This is the main ROS 2 package you will be adding your solutions to.

Then run the first run script to set up your workspace.
```bash
cd crazyflies/crazyflies_sim
./first_run.sh
```

### Step 4: Build the packages

While inside the container:

```bash
cd ~/ros2_ws
chmod +x ~/ros2_ws/src/mini_hero_pkg/scripts/process_image.py
colcon build --packages-select crazyflies_sim mini_hero_pkg --symlink-install --merge-install
source install/setup.bash
```

### Step 5: Start the simulation and the provided script

```bash
cd_crazyflies_sim
./start.sh
```

You can open a new terminal by running the following in a new terminal if you container is running:
`docker exec -i crazyflies_sim_cont` or by opening a new tmux pane by pressing ctrl+b then c.

The main file that you need to modify is located in `mini_hero_pkg/scripts/process_image.py`.

```bash
ros2 run mini_hero_pkg process_image.py
```


### Step 6: Visualize on rviz

Open rviz by running `rviz2` to visualize the original `/cf_1/image` and processed `/processed_image` images.


### Additional resources 

- OpenCV: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- Aruco detection examples: https://www.geeksforgeeks.org/computer-vision/detecting-aruco-markers-with-opencv-and-python-1/


## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)  
3. [Installation & Setup](#installation--setup)
4. [Quick Start Guide](#quick-start-guide)
5. [ROS2 Interface Reference](#ros2-interface-reference)
6. [Configuration & Customization](#configuration--customization)
7. [Troubleshooting](#troubleshooting)

---

---

## Prerequisites

1. **Install Docker**

2. **Enable GUI applications in Docker**:
   ```bash
   xhost +local:docker

   # To make it permanent
   echo "xhost +local:docker > /dev/null" >> ~/.profile
   ```

3. **NVIDIA GPU Support** (if available):
   Follow [NVIDIA Container Toolkit installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

---

## Installation & Setup

### Step 1: Situate yourself in the crazyflies sim folder
```bash
cd crazyflies/crazyflies_sim
```

### Step 2: Build Docker Environment
```bash
# Enable Docker BuildKit
export DOCKER_BUILDKIT=1

# Build the simulation environment (this can take a few minutes)
docker build --ssh default -t crazyflies_sim_img .
```


### Step 3: Setup Your Development Package

If you want to develop your own ROS2 package and have it available inside the container:

1. **Create your local ROS2 package** (outside the container in your local workspace)

2. **Bind your package to the container**:
   Edit the `first_run.sh` file and add your volume mount:
   ```bash  
   # Add this line in the docker run command (before the last backslash):
   --volume "/path/to/your/local/workspace/<package_name>:/root/ros2_ws/src/<package_name>:rw" \
   ```

### Step 4: Container Setup
To set up and start the container:

```bash
# Create and start container
./first_run.sh

# For subsequent use
docker start -i crazyflies_sim_cont

# To open another terminal in a already running container
docker exec -i crazyflies_sim_cont
```

### Step 5: Ensure your local package binding is successful and build

1. **Navigate to your package**:
   ```bash
   cd /root/ros2_ws/src/<your_package_name>
   ```
   Your local files should be visible here!

3. **Build your package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select <your_package_name> --symlink-install --merge-install
   source install/setup.bash
   ```

4. **Edit files to verify binding**:
   - Edit your files on your local machine using IDE
   - Changes are immediately reflected in the container files

**⚠️ Important**: If you create packages inside the container without volume binding, they will be lost when the container is removed! Always backup your work or use volume binding for persistent development.

---

## Quick Start Guide

### Launch Simulation

1. **Enter the container**
   ```bash
   docker start -i crazyflies_sim_cont
   ```

2. **Navigate to startup directory**:
   ```bash
   cd_crazyflies_sim  # This is a pre-configured alias
   # OR manually: cd /root/ros2_ws/src/crazyflies_sim/startup
   ```

3. **Start the simulation**:
   ```bash
   ./start.sh
   ```

4. **What you'll see**:
   - Gazebo simulator window with UAV(s) and ArUco markers
   - RViz visualization showing UAV poses and camera feeds

5. **Run your code**:
   - In another terminal open a new docker terminal:
    ```bash
    docker exec -i crazyflies_sim_cont
    ```
   - Run your nodes/launch files

### Understanding the Default Setup
- **1 CrazyFlie UAV** spawned at origin
- **2 ArUco markers** placed in the environment  
- **Empty world** environment (no obstacles)
- **All ROS2 topics and services** active and ready

---

## ROS2 Interface Reference

> **Note**: Replace `X` with UAV number (e.g., `cf_1`, `cf_2`, etc.)

### Published Topics (UAV → Your Code)

#### Essential State Information
```bash
cf_X/pose                    # geometry_msgs/msg/PoseStamped
cf_X/odom                    # nav_msgs/msg/Odometry  
```

#### Camera & Sensors  
```bash
cf_X/camera_info             # sensor_msgs/msg/CameraInfo
cf_X/image                   # sensor_msgs/msg/Image
```

### Subscribed Topics (Your Code → UAV)

#### Recommended 
```bash
cf_X/cmd_hover               # crazyflie_interfaces/msg/Hover
```

#### Advanced Control
```bash
cf_X/cmd_attitude_setpoint   # crazyflie_interfaces/msg/AttitudeSetpoint
cf_X/cmd_full_state          # crazyflie_interfaces/msg/FullState
cf_X/cmd_vel                 # geometry_msgs/msg/Twist (if using vel_mux)
cf_X/cmd_position            # crazyflie_interfaces/msg/Position
```

### Useful Services

#### Flight Control
```bash
cf_X/takeoff                 # crazyflie_interfaces/srv/Takeoff
cf_X/land                    # crazyflie_interfaces/srv/Land
cf_X/go_to                   # crazyflie_interfaces/srv/GoTo
cf_X/emergency               # std_srvs/srv/Empty
```

### Recommendations

For smooth hardware transition, use relative control methods (`cmd_hover`) rather than absolute positioning, as GPS/motion capture may not be available on real hardware. Assume you only know absolute z position and orientation of the UAV and can publish velocities in the UAV frame.


---

## Configuration & Customization

### Number of UAVs
Edit `/startup/_setup.sh`:
```bash
export NUM_ROBOTS=3  # Change to desired number (1-8)
```
Note that you also need to modify crazyflies.yaml file in the container to set enable flag for more crazyflies.

### UAV Starting Positions
Edit `/launch/UAV_spawn_list/positions.txt`:
```
# Format: x y z yaw
0.0 0.0 0.0 0.0    # UAV 1
1.0 0.0 0.0 0.0    # UAV 2  
0.0 1.0 0.0 0.0    # UAV 3
```

### ArUco Marker Configuration
Edit `/worlds/empty.sdf` to add/modify markers:
```xml
<include>
    <pose>2.0 2.0 1.5 0 0 1.57</pose>  <!-- x y z roll pitch yaw -->
    <uri>model://aruco_markers/aruco_marker_1</uri>
</include>
```

### ArUco Detection Parameters
- **Dictionary**: DICT_5X5_250 (OpenCV ArUco)
- **Marker Size**: 0.25m (simulation) - parameterize for hardware transition
- **IDs**: 1-5 (expandable)

---

### IDE Integration
**VS Code Users**: Install the [Dev Containers extension](https://code.visualstudio.com/docs/remote/containers) to develop directly inside the container:
1. Start container: `docker start -i crazyflies_sim_cont`
2. VS Code: `Ctrl+Shift+P` → "Dev Containers: Attach to Running Container"

---

## Troubleshooting

### Container Won't Start
```bash
# Check if Docker daemon is running
sudo systemctl status docker

# Reset container if corrupted
docker stop crazyflies_sim_cont
docker rm crazyflies_sim_cont # This is a destructive action! Make sure you packages are backup up and any changes inside the container have been copied to your local filesystem!
./first_run.sh
```

### No GUI Display
```bash
# Re-enable X11 forwarding
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY
```

### ROS2 Topics Not Visible
```bash
# Source the workspace
source /root/ros2_ws/install/setup.bash

# Check if nodes are running
ros2 node list

# Verify topic publication
ros2 topic list
ros2 topic echo /cf_1/pose
```

---

## Additional Resources

- **CrazyFlie Documentation**: [Bitcraze Docs](https://www.bitcraze.io/documentation/)
- **ROS2 Tutorials**: [ROS2 Learning Resources](https://docs.ros.org/en/humble/Tutorials.html)
- **ArUco Detection**: [OpenCV ArUco Guide](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
---

*This simulation environment is based on the ICUAS Competition codebase. Original version: [larics/icuas25_competition](https://github.com/larics/icuas25_competition)*