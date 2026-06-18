# Autoware humble — full perception / planning / control stack + Autoware bridge
# No CARLA binary or Python client — pure Autoware + project packages + bridge.
FROM ghcr.io/autowarefoundation/autoware:20240618-devel-cuda-amd64

ARG USERNAME=carla
ARG USER_UID=1000
ARG ROS_DISTRO=humble

ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_VISIBLE_DEVICES=all

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ=America/New_York
ARG DEBIAN_FRONTEND=noninteractive

# ROS 2 apt signing key expired in 2025; remove the old source entry first to avoid
# a Signed-By conflict between the base image's keyring-file reference and the new
# inline-key format installed by ros2-apt-source.deb.
RUN rm -f /etc/apt/sources.list.d/ros2*.list \
          /etc/apt/sources.list.d/ros2*.sources \
    && ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F "tag_name" | awk -F'"' '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb \
       "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb

RUN apt-get update && apt-get install -yq --no-install-recommends \
    sudo tzdata wget git curl unzip libvulkan1 \
    && ln -fns /usr/share/zoneinfo/${TZ} /etc/localtime \
    && dpkg-reconfigure -f noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_UID}" "${USERNAME}" \
    && useradd --create-home --shell /bin/bash \
       -g "${USERNAME}" -G sudo,video,audio,render "${USERNAME}" --uid "${USER_UID}" \
    && echo "${USERNAME}:${USERNAME}" | chpasswd \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

USER ${USERNAME}
WORKDIR /home/${USERNAME}

RUN python3 -m pip install -U pip "setuptools<71.0.0" wheel

# ── ROS2 system deps ─────────────────────────────────────────────────────────
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-tf2-tools" \
    "ros-${ROS_DISTRO}-tf-transformations" \
    "ros-${ROS_DISTRO}-rqt-tf-tree" \
    "ros-${ROS_DISTRO}-tf2-geometry-msgs" \
    "ros-${ROS_DISTRO}-derived-object-msgs" \
    "ros-${ROS_DISTRO}-ackermann-msgs" \
    "ros-${ROS_DISTRO}-robot-localization" \
    "ros-${ROS_DISTRO}-imu-tools" \
    "ros-${ROS_DISTRO}-nav2-msgs" \
    "ros-${ROS_DISTRO}-navigation2" \
    "ros-${ROS_DISTRO}-nav2-bringup" \
    libpcl-dev \
    && sudo rm -rf /var/lib/apt/lists/*

# ── ROS2 workspace ───────────────────────────────────────────────────────────
ENV BUILD_HOME=/home/${USERNAME}/carla_ros_ws

RUN mkdir -p "${BUILD_HOME}/src"

RUN cd "${BUILD_HOME}/src" \
    && git clone https://github.com/privvyledge/autonomous_driving_simulators.git \
       -b ros2_launch \
    && git clone https://github.com/Robotics010/carla_autoware_bridge.git \
       -b feature/add-humble-support \
    && git clone https://github.com/astuff/astuff_sensor_msgs.git -b master \
    && git clone https://github.com/Robotics010/carla_tesla_model3_description.git

ENV PYTHONWARNINGS="ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,ignore:::setuptools.command.develop"
ENV CCACHE_DIR=/home/${USERNAME}/.ccache

ARG SKIPPED_ROSDEP_KEYS="k4a lio_sam fast_lio rf2o_laser_odometry cmake_modules pacmod3_msgs python3-open3d open3d librealsense2 realsense2_camera libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"

RUN cd "${BUILD_HOME}" \
    && sudo apt-get update \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y -q \
       --os=ubuntu:jammy --rosdistro="${ROS_DISTRO}" \
       --skip-keys "${SKIPPED_ROSDEP_KEYS}" \
    && sudo rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd "${BUILD_HOME}" \
    && colcon build --symlink-install \
       --event-handlers console_direct+ \
       --base-paths src \
       --cmake-args \
         ' -Wno-dev' ' --no-warn-unused-cli' \
         ' -DCMAKE_BUILD_TYPE=Release' \
         ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
         ' -DBOOST_BIND_GLOBAL_PLACEHOLDERS=1' \
         ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
         ' -DDOWNLOAD_ARTIFACTS=ON'

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ${BUILD_HOME}/install/setup.bash" >> ~/.bashrc \
    && echo "sudo sysctl -w net.core.rmem_max=2147483647" >> ~/.bashrc \
    && echo "sudo sysctl -w net.ipv4.ipfrag_time=3" >> ~/.bashrc \
    && echo "sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728" >> ~/.bashrc \
    && echo "sudo ip link set lo multicast on" >> ~/.bashrc

WORKDIR /home/${USERNAME}
