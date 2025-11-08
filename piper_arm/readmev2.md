# Agilex PiPER docker instructions

1. **Install Docker**
    
    If you already did this in Crazyflie setup then skip.

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

1. Situate yourself in the piper_arm folder
 
    ```bash
    cd {path_to}/undergrad_proj_larics_2025/piper_arm
    ```

2. Build Docker Environment
    ```bash
    # Enable Docker BuildKit
    export DOCKER_BUILDKIT=1

    # Build the simulation environment (this can take a few minutes)
    docker build --ssh default -t piper_arm_img Dockerfile
    ```

3. Run the container.
    ```bash
    ./first_run.sh
    ```

# Subsequent use
1. To start docker use command:
    ```bash
    docker start -i crazyflies_sim_cont
    ```

2. To open another terminal in a already running container
    ```bash
    docker exec -i crazyflies_sim_cont
    ```
