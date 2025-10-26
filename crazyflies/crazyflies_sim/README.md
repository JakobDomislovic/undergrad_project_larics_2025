# Crazyflies in Simulation

This code is a modified version of the codebase used for the ICUAS Competition Simulation Phase. Original version can be found [here](https://github.com/larics/icuas25_competition).

This simulation environment has been renamed and adapted for general CrazyFlie multi-agent simulation research.

## Instructions

### Docker Prerequisites

Make sure you have completed the docker installation steps and tutorials.

Docker containers are intended to run inside your terminal. In other words, you won't see a desktop like in regular virtual machines. However, graphical applications can still run in the container if you give them permission. To do that, execute

```bash
xhost +local:docker
```

To avoid having to do this every time, we can add that command to our `.profile` file which executes on every login.

```bash
echo "xhost +local:docker > /dev/null" >> ~/.profile
```

If you have an NVIDIA GPU, please install `nvidia-container-toolkit` by following [these instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).


### Set up

1. Navigate to the `crazyflies_sim` folder.

2. Add  to  `~/.bashrc` and source it, or type in the current terminal:

```
export DOCKER_BUILDKIT=1
```

3. Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
docker build --ssh default -t crazyflies_sim_img .
```

4. Bind your ros2 package folder to the container:

Navigate to `first_run.sh` file, and in the docker run command add the argument:

```bash
--volume "<your_package_local_path:/root/CrazySim/ros2_ws/src/meta_packages_hero:rw" \
```


5. Setup the container
```
./first_run.sh
```

This will create docker container crazyflies_sim_cont and position you into the container.

The instructions above are only one time for the first setup.

### Running 

For subsequent use of the container, you can use:

```bash
docker start -i crazyflies_sim_cont
```

Start another bash shell in the already running crazyflies_sim_cont container:
```bash 
docker exec -it crazyflies_sim_cont bash
```

If you ctrl+c in the terminal where you ran `docker start` command, the container will be automatically closed. But you can force stop containers by running:

```bash
docker stop crazyflies_sim_cont
```


The containers `crazyflies_sim_cont` consists of packages for Crazyflies simulator [CrazySim](https://github.com/gtfactslab/CrazySim). General information about Crazyflies can be found [here](https://www.bitcraze.io/products/crazyflie-2-1/).

> [!NOTE]
> The ros2 workspace is located in /root/CrazySim/ros2_ws

### RUN EMPTY WORLD EXAMPLE

Once inside the container, navigate to `/root/CrazySim/ros2_ws/src/crazyflies_sim/startup` (you can use alias `cd_crazyflies_sim` that will place you in crazyflies_sim package). Start the example: 

```
./start.sh
```

If needed, make startup script executable with `chmod +x start.sh`. It starts the example with 5 Crazyflies and 5 aruco markers in an empty world. 


## Tips

**VS Code** - If you normally use VS Code as your IDE, you can install [Dev Containers](https://code.visualstudio.com/docs/remote/containers#_sharing-git-credentials-with-your-container) extension which will allow you to continue using it inside the container. Simply start the container in your terminal (`docker start -i crazyflies_sim_cont `) and then attach to it from the VS code (open action tray with `Ctrl+Shift+P` and select `Dev Containers: Attach to Running Container`).

## TODOs

- [ ] Add additional documentation about how to change robot/aruco poses