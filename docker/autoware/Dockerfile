# Autoware humble — full perception / planning / control stack
# No CARLA binary or Python client — pure Autoware + project packages.
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

# ── MPC Toolchain ────────────────────────────────────────────────────────────
RUN python3 -m pip install --no-cache-dir \
    "do-mpc" "casadi==3.6.6" \
    "transforms3d==0.4.2" "scipy==1.12.0" \
    networkx simple-pid

# Acados — model predictive control library with C code generation
ARG ACADOS_NUM_THREADS=6
ARG ACADOS_OPENMP=ON
ENV ACADOS_ROOT=/home/${USERNAME}/sdks/acados
ENV ACADOS_PATH=${ACADOS_ROOT}
ENV ACADOS_SOURCE_DIR=${ACADOS_ROOT}

RUN mkdir -p /home/${USERNAME}/sdks \
    && cd /home/${USERNAME}/sdks \
    && git clone https://github.com/acados/acados.git \
    && cd acados \
    && git submodule update --recursive --init \
    && mkdir build && cd build \
    && cmake \
        -DACADOS_WITH_QPOASES=ON \
        -DACADOS_WITH_OSQP=ON \
        -DACADOS_INSTALL_DIR="${ACADOS_ROOT}" \
        -DCMAKE_BUILD_TYPE=Release \
        "-DACADOS_NUM_THREADS=${ACADOS_NUM_THREADS}" \
        "-DACADOS_WITH_OPENMP=${ACADOS_OPENMP}" .. \
    && make -j"$(($(nproc)-1))" \
    && sudo make install -j"$(($(nproc)-1))" \
    && python3 -m pip install --use-pep517 --user \
        -e "${ACADOS_ROOT}/interfaces/acados_template"

# tera_renderer — code generation backend for Acados Python interface
# apt rustc in Ubuntu 22.04 is 1.75 which lacks edition2024 (needs 1.85+); use rustup instead.
# mkdir -p is required because cmake install does not create the bin/ directory.
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y \
    && mkdir -p "${ACADOS_ROOT}/bin" \
    && cd "${ACADOS_ROOT}/bin" \
    && git clone https://github.com/acados/tera_renderer.git \
    && cd tera_renderer \
    && . "$HOME/.cargo/env" \
    && cargo build --verbose --release \
    && cp target/release/t_renderer "${ACADOS_ROOT}/bin"

ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ACADOS_ROOT}/lib"

# ── ROS2 system deps ─────────────────────────────────────────────────────────
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-tf2-tools" \
    "ros-${ROS_DISTRO}-tf-transformations" \
    "ros-${ROS_DISTRO}-rqt-tf-tree" \
    "ros-${ROS_DISTRO}-tf2-geometry-msgs" \
    "ros-${ROS_DISTRO}-derived-object-msgs" \
    "ros-${ROS_DISTRO}-ackermann-msgs" \
    "ros-${ROS_DISTRO}-slam-toolbox" \
    "ros-${ROS_DISTRO}-robot-localization" \
    "ros-${ROS_DISTRO}-imu-tools" \
    "ros-${ROS_DISTRO}-nav2-msgs" \
    "ros-${ROS_DISTRO}-navigation2" \
    "ros-${ROS_DISTRO}-nav2-bringup" \
    "ros-${ROS_DISTRO}-rtabmap-odom" \
    "ros-${ROS_DISTRO}-rtabmap-slam" \
    "ros-${ROS_DISTRO}-rtabmap-launch" \
    "ros-${ROS_DISTRO}-rtabmap-viz" \
    libpcl-dev \
    && sudo rm -rf /var/lib/apt/lists/*

# ── ROS2 workspace ───────────────────────────────────────────────────────────
ENV BUILD_HOME=/home/${USERNAME}/carla_ros_ws

RUN mkdir -p "${BUILD_HOME}/src"

RUN cd "${BUILD_HOME}/src" \
    && git clone https://github.com/privvyledge/autonomous_driving_simulators.git \
       -b ros2_launch \
    && git clone https://github.com/privvyledge/trajectory_following_ros2.git \
    && git clone https://github.com/f1tenth/teleop_tools.git -b foxy-devel \
    && git clone https://github.com/f1tenth/ackermann_mux.git -b foxy-devel \
    && git clone https://github.com/ros-teleop/twist_mux.git -b rolling

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
