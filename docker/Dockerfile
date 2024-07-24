# todo: use a different base image, i.e one that does not depend on Autoware
FROM ghcr.io/autowarefoundation/autoware:20240618-devel-cuda-amd64

# ARGS
ARG USERNAME=carla
ENV USERNAME=${USERNAME}
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ARG PYTHON_MAJOR=3
ARG PYTHON_MINOR=10
ARG PYTHON_PATCH=12
ARG UBUNTU_RELEASE_YEAR=22
ARG CUDA_MAJOR=12
ARG CUDA_MINOR=3
ARG CUDA_PATCH=2
ARG CUDNN_VERSION=9
ARG CARLA_MAJOR=0
ARG CARLA_MINOR=9
ARG CARLA_PATCH=15

ENV CARLA_MAJOR=${CARLA_MAJOR}
ENV CARLA_MINOR=${CARLA_MINOR}
ENV CARLA_PATCH=${CARLA_PATCH}


# Set up the shell
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ=America/New_York
ENV LOGNAME=root
ARG DEBIAN_FRONTEND=noninteractive
ARG DEBCONF_NONINTERACTIVE_SEEN=true

ARG ROS_VERSION="ROS2"
ARG ROS_DISTRO="humble"
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_VISIBLE_DEVICES=all

# Install Sudo
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -yq sudo tzdata && \
    ln -fns /usr/share/zoneinfo/${TZ} /etc/localtime && echo $TZ > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata

# Install dependencies
RUN sudo apt-get update -y && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    sudo \
    git \
    curl \
    wget \
    less \
    zstd \
    udev \
    unzip \
    build-essential \
    apt-transport-https \
    software-properties-common \
    lsb-release \
    gnupg2 \
    cmake \
    pkg-config \
    libpython3-dev \
    python${PYTHON_MAJOR}-dev \
    python${PYTHON_MAJOR} \
    python${PYTHON_MAJOR}-venv \
    python${PYTHON_MAJOR}.${PYTHON_MINOR} \
    python${PYTHON_MAJOR}.${PYTHON_MINOR}-venv \
    python${PYTHON_MAJOR}-pip \
    python${PYTHON_MAJOR}-setuptools \
    libpng-dev \
    libtiff5-dev \
    libjpeg-dev \
    libturbojpeg-dev \
    libomp5 \
    xdg-user-dirs \
    xdg-utils \
    alsa-utils \
    libvulkan1 && \
    sudo rm -rf /var/lib/apt/lists/*

# Set Python3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1
RUN python3 -m pip install -U pip setuptools wheel

## Fix ALSA Lib issues (https://github.com/carla-simulator/carla/issues/2820#issuecomment-624093857)
#RUN echo pcm.!default { type plug slave.pcm "null" } >> /etc/asound.conf && \
#    echo pcm.!default { type plug slave.pcm "null" } >> /home/${USER}/.asoundrc

# Create a non root user
RUN useradd --create-home --shell /bin/bash -g root -G sudo,video ${USERNAME} && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo,video $USERNAME && \
        usermod  --uid $USER_UID $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Install Carla
RUN mkdir -p /home/${USERNAME}/Downloads/ && cd /home/${USERNAME}/Downloads/ && \
    wget https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}.tar.gz && \
    mkdir -p /home/${USERNAME}/carla_simulator && tar -xvzf CARLA_${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}.tar.gz -C /home/${USERNAME}/carla_simulator && \
    rm CARLA_${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}.tar.gz && \
#    python3 -m pip install "carla==${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}" && \
    wget https://github.com/gezp/carla_ros/releases/download/carla-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}-ubuntu-${UBUNTU_RELEASE_YEAR}.04/carla-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-linux_x86_64.whl -O /home/${USERNAME}/carla_simulator/PythonAPI/carla/dist/carla-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-linux_x86_64.whl && \
    wget https://github.com/gezp/carla_ros/releases/download/carla-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}-ubuntu-${UBUNTU_RELEASE_YEAR}.04/carla-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}-py${PYTHON_MAJOR}.${PYTHON_MINOR}-linux-x86_64.egg -O /home/${USERNAME}/carla_simulator/PythonAPI/carla/dist/carla-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}-py${PYTHON_MAJOR}.${PYTHON_MINOR}-linux-x86_64.egg && \
    python3 -m pip install /home/${USERNAME}/carla_simulator/PythonAPI/carla/dist/carla-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-linux_x86_64.whl

RUN cd /home/${USERNAME}/carla_simulator && /bin/bash ./ImportAssets.sh

# Install Scenario Runner
RUN cd /home/${USERNAME} && \
    wget https://github.com/carla-simulator/scenario_runner/archive/refs/tags/v${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}.zip && \
    unzip v${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}.zip && rm v${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}.zip && \
    python3 -m pip install xmlschema==1.1.3

ENV CARLA_ROOT=/home/${USERNAME}/carla_simulator
ENV PYTHONPATH=${PYTHONPATH}:${CARLA_ROOT}/PythonAPI/carla/dist/carla-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}-py${PYTHON_MAJOR}.${PYTHON_MINOR}-linux-x86_64.egg
ENV PYTHONPATH=${PYTHONPATH}:${CARLA_ROOT}/PythonAPI/carla
ENV SCENARIO_RUNNER_ROOT=/home/${USERNAME}/scenario_runner-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}
ENV SCENARIO_RUNNER_PATH=/home/${USERNAME}/scenario_runner-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}

## todo: delete the lines below after testing ENVs
RUN echo "export PYTHONPATH=${PYTHONPATH}:${CARLA_ROOT}/PythonAPI/carla/dist/carla-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}-py${PYTHON_MAJOR}.${PYTHON_MINOR}-linux-x86_64.egg" >> /home/${USERNAME}/.bashrc && \
    echo "export PYTHONPATH=${PYTHONPATH}:${CARLA_ROOT}/PythonAPI/carla" >> /home/${USERNAME}/.bashrc && \
    echo "export SCENARIO_RUNNER_ROOT=/home/${USERNAME}/scenario_runner-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}" >> /home/${USERNAME}/.bashrc && \
    echo "export SCENARIO_RUNNER_PATH=/home/${USERNAME}/scenario_runner-${CARLA_MAJOR}.${CARLA_MINOR}.${CARLA_PATCH}" >> /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}/carla_simulator

## Usage
##### Test python installation: python3 -c "import carla"
### SDL_VIDEODRIVER=offscreen or x11.  -prefernvidia or -vulkan or -opengl. -RenderOffScreen. -nosound
### if running into ALSA issues, use -nosound
### to keep a fixed frame rate: -benchmark -fps=30
### -carla-server-timeout=10000ms
### -quality-level=Low (Low or Epic)
### todo: docker run -p 2000-2002:2000-2002 -it --rm --name carlasim --privileged --gpus all --net=host -e DISPLAY=$DISPLAY -e  SDL_VIDEODRIVER=x11 -v /tmp/.X11-unix:/tmp/.X11-unix:rw carlasim/carla:0.9.14 /bin/bash ./CarlaUE4.sh -vulkan


# Install AWSIMLABs
RUN mkdir -p /home/${USERNAME}/Downloads/ && cd /home/${USERNAME}/Downloads/ && \
    wget https://github.com/autowarefoundation/AWSIM-Labs/releases/download/v1.1.1/awsim_labs_v1.1.1.zip && \
    unzip awsim_labs_v1.1.1.zip && rm awsim_labs_v1.1.1.zip && \
    mv awsim_labs_v1.1.1 /home/${USERNAME}/awsim_simulator && \
    cd /home/${USERNAME}/awsim_simulator && chmod +x awsim_labs.x86_64

## Usage
# cd /home/${USERNAME}/awsim_simulator && ./awsim_labs.x86_64

# Install ROS2 Humble (optional: if not using an Autoware or ROS base image)
ENV LANG=en_US.UTF-8
#RUN sudo apt update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
#    locales && \
#    sudo locale-gen en_US en_US.UTF-8  && \
#    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8  && \
#    export LANG=en_US.UTF-8  && \
#    DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends software-properties-common && \
#    sudo add-apt-repository -y universe && \
#    sudo apt update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends curl && \
#    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
#    sudo apt update && sudo apt upgrade -y && \
#    sudo apt update && DEBIAN_FRONTEND="noninteractive" sudo apt install -y python3-rosdep ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-perception ros-dev-tools && \
#    DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-desktop-full && \
#    sudo rm -rf /var/lib/apt/lists/* && \
#    source /opt/ros/${ROS_DISTRO}/setup.bash && \
#    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
#    sudo apt upgrade -y && \
#    sudo rosdep init && \
#    rosdep update

# Install ROS2 Bridges
ARG ROS2_WS=carla_ros_ws
ARG ROS2_WS=${ROS2_WS}
ARG BUILD_HOME=/home/${USERNAME}/${ROS2_WS}
ENV BUILD_HOME=${BUILD_HOME}
RUN mkdir -p ${BUILD_HOME}/src

RUN sudo apt update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-derived-object-msgs ros-${ROS_DISTRO}-tf-transformations

# simple-id==1.0.1
RUN python3 -m pip install "numpy==1.23.5" pygame transforms3d "setuptools==65.7.0" simple-pid networkx pexpect

RUN cd /home/${USERNAME}/Downloads/ && \
    wget https://raw.githubusercontent.com/privvyledge/ros-bridge/feature/add-humble-support/carla_spawn_objects/config/objects.json && \
    wget https://raw.githubusercontent.com/privvyledge/ros-bridge/feature/add-humble-support/carla_spawn_objects/config/objects_autoware_tesla_model3.json && \
    wget https://gist.githubusercontent.com/soumya997/0b7a43fe23d2549aafe19972349664e1/raw/ecac931ac03ef87bde38d431b47e765fdf42ff0a/carla_ros.rviz

### Usage
# ./CarlaUE4.sh -prefernvidia -nosound -benchmark -fps=60
# ros2 launch carla_ros_bridge carla_ros_bridge.launch.py synchronous_mode:=True town:=Town03 timeout:=10
# ros2 launch carla_spawn_objects carla_example_ego_vehicle.launch.py spawn_sensors_only:=False objects_definition_file:=/home/carla/Downloads/objects.json
# rviz2 -d /home/carla/Downloads/carla_ros.rviz

### Maps (https://carla.readthedocs.io/en/latest/core_map/#non-layered-maps | https://github.com/carla-simulator/carla/blob/master/Docs/map_town03.md)

# Install Autoware
RUN sudo rm -rf /autoware && \
    cd ${BUILD_HOME}/src && \
#    git clone https://github.com/privvyledge/autoware.f1tenth.git autoware && cd autoware && \
#    chmod +x install_autoware_dependencies.sh && ./install_autoware_dependencies.sh && \
#    cd .. && rm -rf autoware && \
    git clone https://github.com/autowarefoundation/autoware.git -b humble && cd autoware && \
    mkdir src  && \
    vcs import src < autoware.repos

USER root
# Use ansible to download Autoware artifacts
RUN sudo apt-get -y update && \
    cd ${BUILD_HOME}/src/autoware && \
    python3 -m pip install --user pipx && \
    python3 -m pipx ensurepath && \
    export PATH="${PIPX_BIN_DIR:=$HOME/.local/bin}:$PATH" && \
    pipx install --include-deps --force "ansible==6.*" && \
    ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml" && \
    ansible-playbook autoware.dev_env.download_artifacts -e "data_dir=/home/${USERNAME}/autoware_data" && \
    python3 -m pip uninstall -y ansible ansible-core && \
    sudo apt-get autoremove -y && rm -rf "$HOME"/.cache && \
    chown -R $USERNAME /home/${USERNAME}/autoware_data

USER ${USERNAME}

# Install Carla ROS2 Bridge
#RUN cd ${BUILD_HOME}/src && \
#    git clone --recurse-submodules https://github.com/gezp/carla_ros.git -b humble-carla-0.9.14

# todo: swith to a repos file (https://github.com/Robotics010/carla_autoware_bridge/blob/feature/add-humble-support/getting-started.md#32-add-repositories-required-for-autoware-and-carla-communication)
# either master or feature/add-humble-support
ARG ROBOTICS010_BRANCH="feature/add-humble-support"
# if issues arise, test using branch "feature/add-humble-support" with Robotics010 repos
RUN cd ${BUILD_HOME}/src && mkdir -p carla && cd carla && \
    git clone --recurse-submodules https://github.com/privvyledge/ros-bridge.git -b ${ROBOTICS010_BRANCH}

# Install Carla-Autoware Bridge
RUN cd ${BUILD_HOME}/src && mkdir -p carla && cd carla && \
    git clone https://github.com/astuff/astuff_sensor_msgs.git -b master && \
    git clone https://github.com/Robotics010/carla_autoware_bridge.git -b ${ROBOTICS010_BRANCH} && \
    git clone https://github.com/Robotics010/carla_tesla_model3_description.git -b master && \
    git clone https://github.com/Robotics010/carla_launch.git -b master && \
    git clone https://github.com/Robotics010/carla_control_launch.git -b master
## Usage
# ros2 launch carla_autoware_bridge carla_autoware_demo.launch.py timeout:=10 port:=2000 view:=true
# ros2 launch carla_launch e2e_simulator.launch.xml map_path:=$HOME/autoware_map/carla-town-1 vehicle_model:=carla_tesla_model3 sensor_model:=sample_sensor_kit

# Download Autoware Carla Assets: HD maps (vector maps, lanelet maps), PCDs, YOLO weights, etc.
RUN mkdir -p ~/autoware_map/carla-town-1 && cd ~/Downloads/ && \
    git clone https://bitbucket.org/carla-simulator/autoware-contents.git && \
    cd autoware-contents && \
    cp maps/point_cloud_maps/Town01.pcd ~/autoware_map/carla-town-1/pointcloud_map.pcd && \
    cp maps/vector_maps/lanelet2/Town01.osm ~/autoware_map/carla-town-1/lanelet2_map.osm

RUN mkdir -p ~/autoware_map/Town01 && cd ~/Downloads/ && \
    cd autoware-contents && \
    cp maps/point_cloud_maps/Town01.pcd ~/autoware_map/Town01/pointcloud_map.pcd && \
    cp maps/vector_maps/lanelet2/Town01.osm ~/autoware_map/Town01/lanelet2_map.osm && \
    cd ~/autoware_map && echo "projector_type: local" >> map_projector_info.yaml

# Download Autoware AWSIMLabs Assets
RUN mkdir -p ~/autoware_map/ && cd ~/Downloads/ && \
    wget https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip && \
    unzip nishishinjuku_autoware_map.zip && rm nishishinjuku_autoware_map.zip && \
    mv nishishinjuku_autoware_map ~/autoware_map/nishishinjuku_autoware_map

#COPY --chown=${USERNAME}:${USERNAME} . /home/${USERNAME}

RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    libpcl-dev

#################################################### (Optional) Setup SLAM toolbox. Use galactic and above (or noetic) to get pose
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-slam-toolbox && \
    sudo rm -rf /var/lib/apt/lists/*
# RUN cd "$BUILD_HOME/src" && git clone https://github.com/SteveMacenski/slam_toolbox.git -b ${ROS_DISTRO}-devel

#################################################### (Optional) Setup Robot localization
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-robot-localization && \
    sudo rm -rf /var/lib/apt/lists/*
# RUN cd "$BUILD_HOME/src" && git clone https://github.com/cra-ros-pkg/robot_localization.git -b ${ROS_DISTRO}-devel

#################################################### (Optional) Setup IMU Filters
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-imu-tools && \
    sudo rm -rf /var/lib/apt/lists/*

#################################################### Setup Autonomous bringup packages.
RUN cd "$BUILD_HOME/src" && git clone https://github.com/privvyledge/f1tenth_launch.git -b ${ROS_DISTRO}-dev && \
    git clone https://github.com/privvyledge/trajectory_following_ros2.git

#################################################### Setup Laser filters/pipeline.
#RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \
#    ros-${ROS_DISTRO}-laser-pipeline ros-${ROS_DISTRO}-laser-filters  && \
#    rm -rf /var/lib/apt/lists/*
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-filters && \
    cd "$BUILD_HOME/src" && git clone https://github.com/ros-perception/laser_filters.git -b ros2

#################################################### Setup depth image to laser scan. No need as its already in the container
#RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
#    ros-${ROS_DISTRO}-depthimage-to-laserscan  && \
#    sudo rm -rf /var/lib/apt/lists/*
# RUN cd "$BUILD_HOME/src" && git clone https://github.com/ros-perception/depthimage_to_laserscan.git -b ros2
## Switch back to the main branch above after this pull request has been merged or any other fix for the QoS issues (https://github.com/ros-perception/depthimage_to_laserscan/pull/76)
RUN cd "$BUILD_HOME/src" && git clone https://github.com/Myzhar/depthimage_to_laserscan.git -b ros2_fix_qos_for_ipc

#################################################### Setup Image Proc (e.g PointClouds from depth or stereo images)
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-image-pipeline && \
    sudo rm -rf /var/lib/apt/lists/*

#-------------------------------------------------
# Setup Mapping packages (LIO-SAM).
# Some packages to consider (https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-maps/open-source-slam/)
# https://github.com/TixiaoShan/LIO-SAM/tree/ros2 (lidar, imu, gps)
# https://github.com/Ericsii/FAST_LIO (lidar, imu)
# https://github.com/rsasaki0109/li_slam_ros2
# https://github.com/rsasaki0109/lidarslam_ros2 (lidar, imu)
#-------------------------------------------------
# LIO_SAM
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    software-properties-common \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-perception-pcl \
    ros-${ROS_DISTRO}-pcl-msgs \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-xacro \
    && sudo add-apt-repository -y ppa:borglab/gtsam-release-4.1 \
    && sudo apt-get update \
    && sudo apt install -y --no-install-recommends libgtsam-dev libgtsam-unstable-dev \
    && sudo rm -rf /var/lib/apt/lists/*

RUN cd "$BUILD_HOME/src" && git clone --branch ros2 https://github.com/privvyledge/LIO-SAM.git

#################################################### Setup RTAB-Map (which also publishes odometry from laser_scan)
RUN cd "$BUILD_HOME/src" && git clone https://github.com/introlab/rtabmap.git && git clone https://github.com/introlab/rtabmap_ros.git -b ${ROS_DISTRO}-devel
#RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
#    ros-${ROS_DISTRO}-rtabmap-odom ros-${ROS_DISTRO}-rtabmap-slam \
#    ros-${ROS_DISTRO}-rtabmap-launch ros-${ROS_DISTRO}-rtabmap-python ros-${ROS_DISTRO}-rtabmap-viz \
#    ros-${ROS_DISTRO}-rtabmap-rviz-plugins ros-${ROS_DISTRO}-grid-map-rviz-plugin && \
#    sudo rm -rf /var/lib/apt/lists/*

#----------------------------------------------
# Setup KISS-ICP support
#----------------------------------------------
RUN cd ${BUILD_HOME}/src && git clone https://github.com/PRBonn/kiss-icp.git

#----------------------------------------------
# Install my image recording package
#----------------------------------------------
RUN cd ${BUILD_HOME}/src && git clone https://github.com/privvyledge/ros_images_to_files.git

#----------------------------------------------
# Install YOLOv8 and corresponding ROS nodes
# YOLOv8 nodes.
# Downgrade numpy to 1.23.5 if np.bool depracation errors occure
# Note that downgrading numpy might cause issues with other python packages, i.e onnxruntime-gpu and tensorflow as they require higher numpy versions. Could use a pytorch only base image instead.
# To export to tensorrt
# Using the command line:
# Using python: python3 -c "from ultralytics import YOLO;model = YOLO('yolov8s.pt');model.export(format='engine', half=True, simplify=True)"

# YOLOv8 ROS Usage. todo: put in launch files:
# images only
# ros2 launch ultralytics_ros tracker.launch.xml debug:=false input_topic:=/camera/camera/color/image_raw yolo_model:=yolov8m-seg.pt
# ros2 launch yolov8_bringup yolov8.launch.py model:=yolov8m-seg.pt input_image_topic:=/camera/camera/color/image_raw # or yolov9
# 3D
# ros2 launch ultralytics_ros tracker_with_cloud.launch.xml debug:=false input_topic:=/camera/camera/color/image_raw yolo_model:=yolov8m-seg.pt camera_info_topic:=/camera/camera/color/camera_info lidar_topic:=/camera/camera/depth/color/points yolo_result_topic:=/yolo/realsense/rgb yolo_3d_result_topic:=/yolo/realsense_3d_result cluster_tolerance:=0.3  voxel_leaf_size:=0.1 min_cluster_size:=100 max_cluster_size:=10000
# ros2 launch yolov8_bringup yolov8_3d.launch.py model:=yolov8m-seg.pt input_image_topic:=/camera/camera/color/image_raw input_depth_topic:=/camera/camera/depth/image_rect_raw input_depth_info_topic:=/camera/camera/depth/camera_info target_frame:=camera_link
#----------------------------------------------
# (optionally) download fonts
ADD https://github.com/ultralytics/assets/releases/download/v0.0.0/Arial.ttf \
    https://github.com/ultralytics/assets/releases/download/v0.0.0/Arial.Unicode.ttf \
    /root/.config/Ultralytics/

# Install ultralytics YOLOv8. Tested with version v8.2.35.
# Onnx Runtime 1.18.0 for python 3.10
# optional python versions. numpy==1.23.5 onnx==1.14.0 urllib3==1.26.18 Bottleneck==1.3.4
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    git-lfs gcc git zip curl htop libgl1 libglib2.0-0 libpython3-dev gnupg g++ libusb-1.0-0 && \
#    mkdir -p /root/.config/Ultralytics && cd /root/.config/Ultralytics && wget https://github.com/ultralytics/assets/releases/download/v0.0.0/Arial.ttf && wget https://github.com/ultralytics/assets/releases/download/v0.0.0/Arial.Unicode.ttf && \
    cd /sdks && git clone https://github.com/ultralytics/ultralytics && \
    cd ultralytics && \
    python3 -m pip install --upgrade pip wheel && \
    python3 -m pip install --no-cache-dir "lap==0.4.0" "typing-extensions>=4.4.0" "onnx==1.14.0" "Bottleneck==1.3.6" "onnxsim>=0.4.33" "onnxslim==0.1.28" "numexpr==2.8.4" "tensorflowjs==3.9.0" && \
    cd /sdks/ultralytics/ && python3 -m pip install --no-cache-dir -e ".[export]" && \
#    cd /sdks/ultralytics/ && python3 -m pip install --no-cache -e . && \
    python3 -m pip install "numpy==1.26.4"
#    export OMP_NUM_THREADS=1 && \

#----------------------------------
# Install YOLOv8 ROS packages
#----------------------------------
RUN cd ${BUILD_HOME}/src && \
    GIT_LFS_SKIP_SMUDGE=1 git clone -b humble-devel https://github.com/Alpaca-zip/ultralytics_ros.git && \
    git clone https://github.com/mgonzs13/yolov8_ros.git && \
    sudo rm -rf /var/lib/apt/lists/*

##-----------------------------------------------
## Install/build Open3D (https://www.open3d.org/docs/release/arm.html). Doesn't work in dusty-nv container but works in Nvidia Isaac ROS container.
## To build, see https://www.open3d.org/docs/release/arm.html#building-open3d-python-wheel-with-docker
## todo: compile from source with CUDA support
##-----------------------------------------------
## Todo: test Open3D and Open3D ML with cuda 12.2 then Install CUDA 11.8 if it fails
##RUN sudo apt update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
##    cuda-11-8 cuda-toolkit-11-8 cuda-compat-11-8 cuda-cudart-11-8 cuda-libraries-11-8
#
RUN python3 -m pip install "open3d==0.18.0"
#RUN sudo apt update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends python3-open3d
## todo: use rislab's fork till the main Open3D repo fixes Jetson CUDA compilation issues
## Todo: test realsense support
#RUN python3 -m pip uninstall open3d && \
#    cd /sdks && \
#    git clone https://github.com/isl-org/Open3D.git && \
#    cd Open3D && \
#    ./util/install_deps_ubuntu.sh assume-yes && \
#    mkdir build && cd build && git clone https://github.com/isl-org/Open3D-ML.git && \
#    export PATH=${PATH}:/usr/local/cuda-12.2/bin && export CUDACXX=/usr/local/cuda-12.2/bin/nvcc && \
#    cmake  \
#      -DBUILD_CUDA_MODULE=ON \
#      -DCUDAToolkit_ROOT=/usr/local/cuda-12.2/lib \
#      -DCMAKE_BUILD_TYPE=Release \
#      -DBUILD_GUI=ON \
#      -DGLIBCXX_USE_CXX11_ABI=OFF \
#      -DBUILD_PYTORCH_OPS=ON \
#      -DBUILD_TENSORFLOW_OPS=OFF \
#      -DBUNDLE_OPEN3D_ML=ON \
#      -DBUILD_LIBREALSENSE=OFF \
#      -DBUILD_JUPYTER_EXTENSION:BOOL=TRUE \
#      -DBUILD_WEBRTC=ON \
#      -DCMAKE_INSTALL_PREFIX=/sdks/open3d_install \
#      -DPython3_ROOT=/usr/bin/python3 \
#      -DOPEN3D_ML_ROOT=Open3D-ML \
#      .. && \
#    make -j$(($(nproc)-1)) && \
#    make install-pip-package && sudo make install

# Silence setup.py and easy_install deprecation warnings caused by colcon and setuptools (https://github.com/colcon/colcon-core/issues/454#issuecomment-1142649390)
ENV PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,ignore:::setuptools.command.develop

WORKDIR $BUILD_HOME
#ARG MAKEFLAGS="-j4 -l4"
ARG SKIPPED_ROSDEP_KEYS="k4a lio_sam fast_lio rf2o_laser_odometry cmake_modules pacmod3_msgs  python3-open3d open3d \
    librealsense2 realsense2_camera libopencv-dev  libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"

# Build ROS2 Workspace
RUN cd ${BUILD_HOME} && \
    sudo apt update && \
    rosdep update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y -q --os=ubuntu:jammy --rosdistro=humble --skip-keys "${SKIPPED_ROSDEP_KEYS}"

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
#    source /autoware/install/setup.bash && \
    cd ${BUILD_HOME} && \
    colcon build --symlink-install --event-handlers console_direct+ --base-paths src --cmake-args \
    ' -Wno-dev' ' --no-warn-unused-cli' \
    ' -DCMAKE_BUILD_TYPE=Release' ' -DCMAKE_EXPORT_COMPILE_COMMANDS=ON' \
    ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
    ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"  -DDOWNLOAD_ARTIFACTS=ON' \
    ' -DRTABMAP_SYNC_MULTI_RGBD=ON' \
    --mixin release compile-commands ccache

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
#    echo "source /autoware/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source ${BUILD_HOME}/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "sudo sysctl -w net.core.rmem_max=2147483647" >> /home/${USERNAME}/.bashrc && \
    echo "sudo sysctl -w net.ipv4.ipfrag_time=3" >> /home/${USERNAME}/.bashrc && \
    echo "sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728" >> /home/${USERNAME}/.bashrc && \
    echo "sudo ip link set lo multicast on" >> /home/${USERNAME}/.bashrc

# Todo: add aliases

WORKDIR /home/${USERNAME}