# Agilex PiPER docker instructions

1. **Install Docker**
    
    If you already did this in Crazyflie setup then skip to *Installation & setup* section.

2. **Enable GUI applications in Docker**:
   ```bash
   xhost +local:docker

   # To make it permanent
   echo "xhost +local:docker > /dev/null" >> ~/.profile
   ```

3. **NVIDIA GPU Support** (if available):
   Follow [NVIDIA Container Toolkit installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

    If you already did this in Crazyflie setup then skip.

## Installation & Setup

1. Go to the piper_arm directory inside repository.
 
    ```bash
    cd {path_to}/undergrad_proj_larics_2025/piper_arm
    ```

2. Build Docker image
    ```bash
    # Enable Docker BuildKit
    export DOCKER_BUILDKIT=1

    # Build the simulation environment (this can take a few minutes)
    docker build --ssh default -t piper_arm_img .
    ```

3. Run the container.
    ```bash
    ./first_run.sh
    ```

# Subsequent use
1. To start docker use command:
    ```bash
    docker start -i piper_arm_cont
    ```

2. To open another terminal in a already running container
    ```bash
    docker exec -it piper_arm_cont bash
    ```

## Run Agilex PiPER simulation

You can either open two terminals and in one start docker container and exec other, or open one container and use **tmux** to open two panes and write next scripts.

To run robot simulation and MoveIt scripts:
```bash
ros2 launch piper_gazebo piper_no_gripper_gazebo.launch.py # in one terminal/pane
ros2 launch piper_no_gripper_moveit piper_moveit.launch.py # in other terminal/pane
```
Check that you are able to control the simulated robot arm by dragging the end effector to a new pose and clicking **Plan & Execute**. The robot arm should move both in RViz and Gazebo.

# Run the full control pipeline on the NUC

First, you need to ensure your devices are connected and the CAN ports configured:
```bash
cd mini_hero/can_scripts
bash can_activate can0 1000000 3-4:1.0
bash can_activate can1 500000 3-7:1.0
```

Now, start the docker container:
```bash
docker start elated_newton
```
You may enter into a container using the following command:
```bash
docker exec -it elated_newton bash
```

In the container run the following in separate terminals:
```bash
ros2 launch piper start_single_piper.launch.py

ros2 launch piper_no_gripper_moveit piper_servo.launch.py

ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
python3 controller_control.py

ros2 run joy joy_node

ros2 launch scout_base scout_base.launch.py
```
