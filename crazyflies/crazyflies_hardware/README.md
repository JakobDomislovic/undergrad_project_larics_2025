# Crazyflie Hardware Environment

This code contains the simulation stack for crazyflies in hardware. It is a modified version of the codebase used for the ICUAS25 Competitions. Original versions can be found [here](https://github.com/larics/icuas25_finals).


## Setup

The overall setup for docker is identical to what you did for the simulation setup. The steps are summarized below:

```bash
cd crazyflies/crazyflies_hardware
export DOCKER_BUILDKIT=1
docker build --ssh default -t crazyflies_hardware_img .
```

Then go to `crazyflies_hardware/first_run.sh` file and ensure the volume paths are set right.

You already know how to start and use the container.

```bash
# Create and start container
./first_run.sh

# For subsequent use
docker start -i crazyflies_sim_cont

# To open another terminal in a already running container
docker exec -it crazyflies_sim_cont bash
```

Build packages if needed:
```bash
cd ~/ros2_ws
colcon build --packages-select <your_package_name> --symlink-install --merge-install
source install/setup.bash
```

## How to run

The main difference between simulation and hardware setups is how you connect to the crazyflie. Here, Crazyradio 2.0 attached to your computer connects to the Crazyflie which has a URI. This is mentioned in `crazyflies_hardware/config/crazyflies_icuas.yaml` file.

The URI will be given to you but you can find it by running `cfclient` in the terminal inside the container. Then press scan. If you cant find the Crazyflie on default address `E7E7E7E7E7`, try changing address it to `E7E7E7E7E4` or `E7E7E7E7E` and scan again. The full URI as mentioned in `crazylies_icuas.yaml` file looks like `radio://0/90/2M/E7E7E7E7E0` where `90` corresponds to communication channel and `E7E7E7E7E0` is the address.

---

### Task 1: Figure out Crazyflie Adress

Based on the above description, try to connect to the crazyflie using `cfclient`. Once you find the address after scanning, press connect. It will show that the Crazyflie is armed and display its battery voltage. This is a quick way to check the battery level as well. Then disconnected.

DO NOT PRESS ANY MOTION COMMANDS! Crazyflie IS ARMED HERE.

---

Now for the image, in simulation the simulation stack provides images coming from the Crazyflie. Here this is achieved by the AI deck. It is a module attached on the top of Crazyflie and it communicates over WiFi. When you start the Crazyflie, two LEDs on the AI deck must light up/blink. If not, turn off Crazyflie, wait a few seconds, then start again.

To get images on your computer, you must connect your laptop to 'Wifi streaming example' WiFi. That way you computer can receive image from the AI deck. The IP of AI deck is mentioned `crazyflies_hardware/crazyflies_hardware/config/aideck_streamer.yaml`. You should set it to `192.168.4.1`. This is the default to connect to the device (AI deck itself). Later we will discuss how to connect to multiple C with AI decks. The received scripts are processed by the aideck_streamer_udp.py scripts to process it and publish it on the `cf_1/image` topic, identical to what you had in simulation.


---

### Task 2: Run the simulation and confirm that you get thw image in RViz

Once you have completed the setup above, run the following commands:

```bash
cd_crazyflies_hardware  # This is a pre-configured alias
# OR manually: cd /root/ros2_ws/src/crazyflies_sim/startup
./start.sh
   ```

---

---

### Task 3: Check AR tag pose estimation in isolation 

You can try your code that you used in simulation but here make sure you disable Crazyflie movements in your code by commenting the part that publishes velocities. Move the AR tag around to see if the direction of proposed velocities as you did for simulation in Rviz.

Once that works you can move to Crazyflie control. Ensure very low velocities are set and have safeguards so for any corner case just land the Crazyflie.

Tip: In Rviz, set `global options -> fixed frame` to `cf_1` so we get everything with respect to `cf_1` and make sure your velocity arrow/direction conforms to this frame of ref.

---