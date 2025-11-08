# Ubuntu 22.04
FROM osrf/ros:humble-desktop

ENV LANG=C.UTF-8
ENV LC_AL=C.UTF-8
ENV ROS2_DISTRO=humble
ARG DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV TZ=Europe/Zagreb
ARG HOME=/root


# Tools useful during development
RUN apt-get update \
    && apt install -y \
    build-essential \
    atop \
    clang \
    cmake \
    cppcheck \
    expect \
    gdb \
    git \
    gnutls-bin \
    libbluetooth-dev \
    libccd-dev \
    libcwiid-dev \
    libfcl-dev \
    libgoogle-glog-dev \
    libspnav-dev \
    libspdlog-dev \
    libusb-dev \
    libboost-thread-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    libboost-regex-dev \
    libboost-program-options-dev \
    libconsole-bridge-dev \
    libpoco-dev \
    libtinyxml2-dev \
    liblz4-dev \
    libbz2-dev \
    uuid-dev \
    liblog4cxx-dev \
    libgpgme-dev \
    libgtest-dev \
    python3-dbg \
    python3-empy \
    python3-numpy \
    python3-setuptools \
    python3-pip \
    python3-venv \
    python3-nose \
    python3-pycryptodome \
    python3-defusedxml \
    python3-mock \
    python3-netifaces \
    python3-gnupg \
    python3-psutil \
    vim \
    tmux \
    tmuxinator \
    nano \
    net-tools \
    iputils-ping \
    xvfb \
    curl\
    jq\
    wget\
    ranger \
    htop \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    rfkill \
    sudo \
    usbutils \
    software-properties-common \
    && add-apt-repository universe \
    && apt-get clean -qq

RUN apt-get update \
    && apt install -y \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-repeat \
    python3-pytest-rerunfailures


# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build

ENV DOCKER_BUILDKIT=1
RUN apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v"
USER root

RUN mkdir -p -m 0600 ~/.ssh/ && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Clone piper workspace

RUN cd ~ && git clone -b humble https://github.com/agilexrobotics/piper_ros.git

# install following dependencies
RUN pip3 install piper_sdk
RUN sudo apt-get update \
    && sudo apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-moveit* \
    ros-humble-control* \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-* \
    ros-humble-gripper-controllers \
    ros-humble-trajectory-msgs \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    python3-rosdep \
    ros-dev-tools

RUN apt-get update &&  apt-get upgrade -y && sudo apt install ros-${ROS2_DISTRO}-desktop -y

RUN apt-get install -y python3-colcon-common-extensions


#Add mcap to bag things in ROS2
RUN VERSION="releases/mcap-cli/v0.0.55" && \
    RELEASE_URL=$(curl -s https://api.github.com/repos/foxglove/mcap/releases | jq -r --arg VERSION "$VERSION" '.[] | select(.tag_name == $VERSION) | .assets[0].browser_download_url') && \
    echo "Downloading release $VERSION from: $RELEASE_URL" && \
    curl -L -o /bin/mcap "$RELEASE_URL" && \
    cd /bin && chmod +x mcap


# Build workspace
WORKDIR $HOME/piper_ros
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --merge-install && \
    source install/setup.bash

# Create aliases 
RUN echo "alias source_ros2=\'source /opt/ros/humble/setup.bash'" >> ~/.bashrc && \
    echo "alias source_ros2ws=\'source ~/piper_ros/install/setup.bash'" >> ~/.bashrc && \
    echo "alias colcon_build=\'colcon build --symlink-install --merge-install'" >> ~/.bashrc


RUN echo "export PATH='$HOME/.local/bin:$PATH'" >> $HOME/.bashrc

RUN apt install libboost-program-options-dev libusb-1.0-0-dev
RUN pip3 install rowan transforms3d
RUN apt-get update &&  apt-get upgrade -y && apt-get install -y \
    ros-${ROS2_DISTRO}-tf-transformations \
    ros-${ROS2_DISTRO}-nav2-map-server \
    ros-${ROS2_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS2_DISTRO}-rosbridge-suite \
    ros-${ROS2_DISTRO}-rosbag2-storage-mcap \ 
    ros-${ROS2_DISTRO}-ros-gz-interfaces \
    ros-${ROS2_DISTRO}-ros-gz-bridge \
    ros-${ROS2_DISTRO}-octomap \
    ros-${ROS2_DISTRO}-octomap-ros \
    ros-${ROS2_DISTRO}-octomap-server \
    ros-${ROS2_DISTRO}-octomap-msgs
RUN apt install -y ros-${ROS2_DISTRO}-ros-gz${GZ_RELEASE}

# setup ros2 environment variables
RUN echo "export ROS_LOCALHOST_ONLY=1" >> $HOME/.bashrc
RUN echo "export ROS_DOMAIN_ID=$(shuf -i 1-101 -n 1)" >> $HOME/.bashrc


WORKDIR $HOME
COPY to_copy/aliases $HOME/.bash_aliases
COPY to_copy/nanorc $HOME/.nanorc
COPY to_copy/tmux $HOME/.tmux.conf
COPY to_copy/ranger $HOME/.config/ranger/rc.conf

USER root
WORKDIR $HOME
CMD ["/bin/bash"]
