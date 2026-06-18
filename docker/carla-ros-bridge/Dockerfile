# CARLA ROS2 Bridge
# Includes: carla_ros_bridge, carla_spawn_objects,
#           carla_waypoint_publisher, carla_ackermann_control
# Does NOT include the full Autoware planning/perception stack or Autoware bridge.
FROM ros:humble-ros-base

ARG CARLA_VERSION=0.9.14
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
    sudo tzdata wget unzip git curl python3-pip python3-wheel \
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
RUN python3 -m pip install --no-cache-dir \
    "https://github.com/gezp/carla_ros/releases/download/carla-${CARLA_VERSION}-ubuntu-${UBUNTU_VERSION}.04/carla-${CARLA_VERSION}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-linux_x86_64.whl"

# Python bridge runtime deps
RUN python3 -m pip install --no-cache-dir \
    "numpy==1.23.5" \
    pygame \
    "transforms3d==0.4.2" \
    "setuptools==65.7.0" \
    simple-pid networkx pexpect \
    "scipy==1.12.0"

# CARLA `agents` PythonAPI package (global route planner used by
# carla_waypoint_publisher) — NOT bundled in the wheel. Sparse-checkout just that
# directory at the matching tag and drop it beside the installed `carla` package.
RUN git clone --depth 1 --filter=blob:none --sparse \
        -b "${CARLA_VERSION}" https://github.com/carla-simulator/carla.git /tmp/carla_src \
    && git -C /tmp/carla_src sparse-checkout set PythonAPI/carla/agents \
    && SITE_PKGS="$(python3 -c 'import carla, os; print(os.path.dirname(os.path.dirname(os.path.realpath(carla.__file__))))')" \
    && cp -r /tmp/carla_src/PythonAPI/carla/agents "${SITE_PKGS}/agents" \
    && rm -rf /tmp/carla_src

# Pin NumPy to the 1.x ABI — NumPy 2.x breaks cv_bridge (built against NumPy 1.x).
RUN python3 -m pip install --no-cache-dir "numpy==1.26.4"

# ROS bridge system deps
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-derived-object-msgs" \
    "ros-${ROS_DISTRO}-tf-transformations" \
    "ros-${ROS_DISTRO}-ackermann-msgs" \
    "ros-${ROS_DISTRO}-rqt-image-view" \
    "ros-${ROS_DISTRO}-rqt-gui-py" \
    "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
    "ros-${ROS_DISTRO}-topic-tools" \
    && sudo rm -rf /var/lib/apt/lists/*

# Build CARLA ROS2 bridge workspace
ENV BUILD_HOME=/home/${USERNAME}/carla_ros_ws

RUN mkdir -p "${BUILD_HOME}/src/carla"

RUN cd "${BUILD_HOME}/src/carla" \
    && git clone --recurse-submodules \
       https://github.com/privvyledge/ros-bridge.git \
       -b "${ROBOTICS010_BRANCH}" \
    && git clone https://github.com/Robotics010/carla_tesla_model3_description.git \
    && git clone https://github.com/Robotics010/carla_launch.git \
    && git clone https://github.com/Robotics010/carla_control_launch.git

# Align carla_ros_bridge's expected client version with the installed client.
RUN find "${BUILD_HOME}/src" -name CARLA_VERSION -type f \
        -exec sh -c 'printf "%s\n" "$1" > "$2"' _ "${CARLA_VERSION}" {} \;

# Copy the local autonomous_driving_simulators package from the build context
# instead of cloning it from GitHub, so local edits are baked into the image.
ENV ADS_SRC="${BUILD_HOME}/src/autonomous_driving_simulators"
COPY --chown=${USERNAME}:${USERNAME} package.xml setup.py setup.cfg requirements.txt "${ADS_SRC}/"
COPY --chown=${USERNAME}:${USERNAME} resource "${ADS_SRC}/resource"
COPY --chown=${USERNAME}:${USERNAME} config "${ADS_SRC}/config"
COPY --chown=${USERNAME}:${USERNAME} launch "${ADS_SRC}/launch"
COPY --chown=${USERNAME}:${USERNAME} data "${ADS_SRC}/data"
COPY --chown=${USERNAME}:${USERNAME} scripts "${ADS_SRC}/scripts"
COPY --chown=${USERNAME}:${USERNAME} rviz "${ADS_SRC}/rviz"
COPY --chown=${USERNAME}:${USERNAME} test "${ADS_SRC}/test"
COPY --chown=${USERNAME}:${USERNAME} autonomous_driving_simulators "${ADS_SRC}/autonomous_driving_simulators"

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
         ' -DCMAKE_BUILD_TYPE=Release'

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ${BUILD_HOME}/install/setup.bash" >> ~/.bashrc \
    && echo "sudo sysctl -w net.core.rmem_max=2147483647" >> ~/.bashrc \
    && echo "sudo sysctl -w net.ipv4.ipfrag_time=3" >> ~/.bashrc \
    && echo "sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728" >> ~/.bashrc \
    && echo "sudo ip link set lo multicast on" >> ~/.bashrc

WORKDIR /home/${USERNAME}
