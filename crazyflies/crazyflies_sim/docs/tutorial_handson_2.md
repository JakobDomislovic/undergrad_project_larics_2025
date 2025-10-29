# Hands-on #2: Crazyflie Simulation Setup
---

In this hands-on, you will set up crazyflies simulation stack and run your first simulation.

## Docker Setup

1. **Install Docker:** Follow the link [here](https://github.com/larics/docker_files/wiki/2.-Installation) to install docker.

2. **Enable GUI applications in Docker**:
   ```bash
   xhost +local:docker

   # To make it permanent
   echo "xhost +local:docker > /dev/null" >> ~/.profile
   ```

3. **NVIDIA GPU Support** (if available):
   Follow [NVIDIA Container Toolkit installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Simulation Docker Setup

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

1. **Create your local ROS2 package** (outside the container in your local workspace). For now you can create an empty folder in this repo with your package name.

2. **Bind your package to the container**:
   Edit the `first_run.sh` file and add your volume mount:
   ```bash  
   # Add this line in the docker run command (before the last backslash):
   --volume "/path/to/your/local/workspace/<package_name>:/root/ros2_ws/src/<package_name>:rw" \
   ```

3. Also modify the path reference to the crazyflies_sim folder:
    ```bash  
    # Replace by your local path
    --volume "/path/to/your/repo/undergrad_project_larics_2025/crazyflies/crazyflies_sim:/root/ros2_ws/src/crazyflies_sim" \
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

Additionally, rebuild the crazyflies_sim package:

```bash
cd ~/ros2_ws
colcon build --packages-select crazyflies_sim --symlink-install --merge-install
source install/setup.bash
```

### Step 5: Ensure your local package binding is successful and build

1. **IDE Setup**
VS Code Users: Install the [Dev Containers extension](https://code.visualstudio.com/docs/remote/containers) to develop directly inside the container. Then press `ctrl+shift+p`, search for `Dev Containers: Attach to Running Container`. After this you should be able to see the container's filesystem in VS code.

2. **Navigate to your package**:
   ```bash
   cd /root/ros2_ws/src/<your_package_name>
   ```
   Your local files should be visible here!

3. **Build your package**:
   ```bash
   cd ~/ros2_ws
   chmod +x src/crazyflies_sim/scripts/*  # makes all files in scripts/ executable
   colcon build --packages-select <your_package_name> --symlink-install --merge-install
   source install/setup.bash
   ```

4. **Edit files to verify binding**:
   - Edit your files on your local machine using IDE
   - Changes are immediately reflected in the container files

**⚠️ Important**: If you create packages inside the container without volume binding, they will be lost when the container is removed! Always use volume binding for persistent development.

## Run the simulation

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

### Published Topics (UAV → Your Code)

1. Run the following command to see the list of available topics:
```bash
ros2 topic list
```

#### Essential State Information
```bash
cf_X/pose                    # geometry_msgs/msg/PoseStamped
cf_X/odom                    # nav_msgs/msg/Odometry  
```

2. Echo the pose topic to see its contents
```bash
ros2 topic echo /cf_1/pose
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


3. Takeoff and land a UAV
```bash
# Takeoff
ros2 service call /cf_1/takeoff crazyflie_interfaces/srv/Takeoff "group_mask: 0
height: 0.5
duration:
  sec: 3
  nanosec: 0"

# Land
ros2 service call /cf_1/land crazyflie_interfaces/srv/Land "group_mask: 0
height: 0.0
duration:
  sec: 0
  nanosec: 0"
```

4. Run a sample python script for takeoff move around a bit and land:
```bash
ros2 run crazyflies_sim control_demo.py
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