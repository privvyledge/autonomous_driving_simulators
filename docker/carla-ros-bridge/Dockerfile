# CARLA ROS2 Bridge + Autoware Bridge
# Includes: carla_ros_bridge, carla_autoware_bridge, carla_spawn_objects,
#           carla_waypoint_publisher, carla_ackermann_control
# Does NOT include the full Autoware planning/perception stack.
FROM ghcr.io/autowarefoundation/autoware:20240618-devel-cuda-amd64

ARG CARLA_VERSION=0.9.15
ARG UBUNTU_VERSION=22
ARG PYTHON_MAJOR=3
ARG PYTHON_MINOR=10
ARG USERNAME=carla
ARG USER_UID=1000
ARG ROS_DISTRO=humble
ARG ROBOTICS010_BRANCH=feature/add-humble-support

ENV CARLA_VERSION=${CARLA_VERSION}
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_VISIBLE_DEVICES=all

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ=America/New_York
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -yq --no-install-recommends \
    sudo tzdata wget unzip git curl \
    && ln -fns /usr/share/zoneinfo/${TZ} /etc/localtime \
    && dpkg-reconfigure -f noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_UID}" "${USERNAME}" \
    && useradd --create-home --shell /bin/bash \
       -g "${USERNAME}" -G sudo,video "${USERNAME}" --uid "${USER_UID}" \
    && echo "${USERNAME}:${USERNAME}" | chpasswd \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# CARLA Python client wheel for Python 3.10 on Ubuntu 22.04
RUN mkdir -p Downloads \
    && wget -q \
       "https://github.com/gezp/carla_ros/releases/download/carla-${CARLA_VERSION}-ubuntu-${UBUNTU_VERSION}.04/carla-${CARLA_VERSION}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-linux_x86_64.whl" \
       -O Downloads/carla.whl \
    && python3 -m pip install --no-cache-dir Downloads/carla.whl \
    && rm Downloads/carla.whl

# Python bridge runtime deps
RUN python3 -m pip install --no-cache-dir \
    "numpy==1.23.5" \
    pygame \
    "transforms3d==0.4.2" \
    "setuptools==65.7.0" \
    simple-pid networkx pexpect \
    "scipy==1.12.0"

# ROS bridge system deps
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-derived-object-msgs" \
    "ros-${ROS_DISTRO}-tf-transformations" \
    "ros-${ROS_DISTRO}-ackermann-msgs" \
    "ros-${ROS_DISTRO}-rqt-image-view" \
    "ros-${ROS_DISTRO}-rqt-gui-py" \
    && sudo rm -rf /var/lib/apt/lists/*

# Build CARLA ROS2 bridge workspace
ENV BUILD_HOME=/home/${USERNAME}/carla_ros_ws

RUN mkdir -p "${BUILD_HOME}/src/carla"

RUN cd "${BUILD_HOME}/src/carla" \
    && git clone --recurse-submodules \
       https://github.com/privvyledge/ros-bridge.git \
       -b "${ROBOTICS010_BRANCH}" \
    && git clone https://github.com/astuff/astuff_sensor_msgs.git -b master \
    && git clone https://github.com/Robotics010/carla_autoware_bridge.git \
       -b "${ROBOTICS010_BRANCH}" \
    && git clone https://github.com/Robotics010/carla_tesla_model3_description.git \
    && git clone https://github.com/Robotics010/carla_launch.git \
    && git clone https://github.com/Robotics010/carla_control_launch.git

ENV PYTHONWARNINGS="ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,ignore:::setuptools.command.develop"

RUN cd "${BUILD_HOME}" \
    && sudo apt-get update \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y -q \
       --os=ubuntu:jammy --rosdistro="${ROS_DISTRO}" \
    && sudo rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd "${BUILD_HOME}" \
    && colcon build --symlink-install \
       --event-handlers console_direct+ \
       --base-paths src \
       --cmake-args \
         ' -Wno-dev' \
         ' -DCMAKE_BUILD_TYPE=Release' \
         ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
         ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"'

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ${BUILD_HOME}/install/setup.bash" >> ~/.bashrc \
    && echo "sudo sysctl -w net.core.rmem_max=2147483647" >> ~/.bashrc \
    && echo "sudo sysctl -w net.ipv4.ipfrag_time=3" >> ~/.bashrc \
    && echo "sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728" >> ~/.bashrc \
    && echo "sudo ip link set lo multicast on" >> ~/.bashrc

WORKDIR /home/${USERNAME}
