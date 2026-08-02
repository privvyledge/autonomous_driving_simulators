# syntax=docker/dockerfile:1
# Custom Nodes humble — GPU accelerated container for control and SLAM nodes
FROM nvidia/cuda:12.2.2-devel-ubuntu22.04

ARG USERNAME=carla
ARG USER_UID=1000
ARG ROS_DISTRO=humble

# NOTE: This container is CARLA-package free. CARLA low-level actuation
# (carla_ackermann_control / carla_twist_to_control) and the CARLA Python client
# live in the carla-ros-bridge container; this one only builds the controllers,
# SLAM, perception and teleop stack from custom.repos.

ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_VISIBLE_DEVICES=all

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ=America/New_York
ARG DEBIAN_FRONTEND=noninteractive

# Update keys, locales, timezone and install development tools
RUN apt-get update && apt-get install -yq --no-install-recommends \
    sudo tzdata wget git curl unzip gnupg2 lsb-release locales iproute2 \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && ln -fns /usr/share/zoneinfo/${TZ} /etc/localtime \
    && dpkg-reconfigure -f noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Setup ROS 2 Humble apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble base, vcstool, and development tools
RUN apt-get update && apt-get install -yq --no-install-recommends \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    python3-wheel \
    python3-setuptools \
    build-essential \
    ccache \
    && rm -rf /var/lib/apt/lists/*

# Setup user
RUN groupadd --gid "${USER_UID}" "${USERNAME}" \
    && (groupadd --system render || true) \
    && useradd --create-home --shell /bin/bash \
       -g "${USERNAME}" -G sudo,video,audio,render "${USERNAME}" --uid "${USER_UID}" \
    && echo "${USERNAME}:${USERNAME}" | chpasswd \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

# Initialize rosdep
RUN rosdep init || true

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# pip installs as the non-root carla user (no venv) default to a USER install, so
# console scripts (yolo, etc.) land in ~/.local/bin. Put it on PATH so the CLIs
# resolve — without this, `import ultralytics` works but `yolo …` is not found.
ENV PATH=/home/${USERNAME}/.local/bin:$PATH

# Setuptools constraint
RUN python3 -m pip install -U pip "setuptools<71.0.0" wheel

# ── MPC Toolchain ────────────────────────────────────────────────────────────
RUN python3 -m pip install --no-cache-dir \
    "do-mpc" "casadi==3.7.2" \
    "transforms3d==0.4.2" "scipy==1.15.3" \
    networkx simple-pid

# Pin NumPy to the 1.x ABI — NumPy 2.x breaks cv_bridge (built against NumPy 1.x).
# NOTE: the ML toolchain below re-pins NumPy again as the final pip action, since
# open3d/ultralytics/torch can drag in NumPy 2.x after this point.
RUN python3 -m pip install --no-cache-dir "numpy==1.26.4"

# ccache: the acados and colcon steps below mount a BuildKit cache at CCACHE_DIR,
# so object files survive across `docker compose build` runs.
# CMAKE_*_COMPILER_LAUNCHER is what actually routes compiles through ccache
# (installing it is not enough).
ENV CCACHE_DIR=/home/${USERNAME}/.ccache
ENV CCACHE_MAXSIZE=5G

# Acados
ARG ACADOS_NUM_THREADS=6
ARG ACADOS_OPENMP=ON
ENV ACADOS_ROOT=/home/${USERNAME}/sdks/acados
ENV ACADOS_PATH=${ACADOS_ROOT}
ENV ACADOS_SOURCE_DIR=${ACADOS_ROOT}

RUN --mount=type=cache,target=/home/${USERNAME}/.ccache,uid=${USER_UID},gid=${USER_UID} \
    mkdir -p /home/${USERNAME}/sdks \
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
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
#        "-DACADOS_NUM_THREADS=${ACADOS_NUM_THREADS}" \
        "-DACADOS_WITH_OPENMP=${ACADOS_OPENMP}" .. \
    && make -j"$(($(nproc)-1))" \
    && sudo make install -j"$(($(nproc)-1))" \
    && python3 -m pip install --use-pep517 --user \
        -e "${ACADOS_ROOT}/interfaces/acados_template"

# tera_renderer
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y \
    && mkdir -p "${ACADOS_ROOT}/bin" \
    && cd "${ACADOS_ROOT}/bin" \
    && git clone https://github.com/acados/tera_renderer.git \
    && cd tera_renderer \
    && . "$HOME/.cargo/env" \
    && cargo build --verbose --release \
    && cp target/release/t_renderer "${ACADOS_ROOT}/bin"

ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ACADOS_ROOT}/lib"

# ── Perception / ML toolchain (Open3D GPU, YOLO, TensorRT) ───────────────────
# Runtime deps for autodriver_image_object_detection (YOLO) and
# autodriver_pointcloud_object_detection (Open3D clustering). None are declared
# in those packages' package.xml, so rosdep won't install them — they live here.
#
# torch/torchvision: CUDA 12.1 wheels (run fine on the 12.2 base); installed
#   before ultralytics so it doesn't pull a non-deterministic/CPU build.
# open3d: the default x86_64 Linux wheel ships WITH CUDA (open3d-cpu is CPU-only),
#   so plain `open3d` is the latest release with GPU support — no source build.
# onnxruntime-gpu + onnx + onnxslim: the .pt -> .onnx leg (and runtime check that
#   ultralytics would otherwise AutoUpdate-install 300MB into ephemeral ~/.local).
#   tensorrt (the .onnx -> .engine builder) is installed SEPARATELY below, as root
#   and pinned to the CUDA-12 line — see that block for why.
#
# OpenCV: ultralytics pulls the GUI `opencv-python` (>=4.6). We swap it for
# `opencv-python-headless` so there is exactly ONE pip cv2 (no namespace clash
# with the apt python3-opencv that cv_bridge links) and ultralytics' runtime
# check_requirements stays satisfied (cv2 -> opencv-python-headless) instead of
# auto-installing the GUI build over the network in this headless container.
RUN python3 -m pip install --no-cache-dir \
        --index-url https://download.pytorch.org/whl/cu121 \
        torch torchvision \
    && python3 -m pip install --no-cache-dir \
        ultralytics open3d onnxruntime-gpu onnx onnxslim \
    && python3 -m pip uninstall -y opencv-python || true \
    && python3 -m pip install --no-cache-dir "opencv-python-headless==4.9.0.80"

# TensorRT (.onnx -> .engine builder) — pinned to the CUDA-12 line.
# The unpinned default pulls TRT 11.x (CUDA-13), whose builder fails CUDA init on a
# 12.x driver: `createInferBuilder ... CUDA initialization failure with error: 35`
# (cudaErrorInsufficientDriver) -> `pybind11::init(): factory function returned nullptr`.
# Installed as ROOT because the `tensorrt` sdist shells out a NESTED `pip install` of
# its cu12 libs/bindings WITHOUT --user, so it needs write access to global
# site-packages (as the non-root carla user that nested install dies Permission denied).
USER root
RUN python3 -m pip install --no-cache-dir "tensorrt==10.7.0"
USER ${USERNAME}

# Re-pin NumPy to the 1.x ABI as the FINAL pip action — open3d/ultralytics/torch
# can pull NumPy 2.x, which breaks cv_bridge (built against NumPy 1.x).
RUN python3 -m pip install --no-cache-dir "numpy==1.26.4"

# ── ROS2 system deps ─────────────────────────────────────────────────────────
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-tf2-tools" \
    "ros-${ROS_DISTRO}-tf-transformations" \
    "ros-${ROS_DISTRO}-rqt-tf-tree" \
    "ros-${ROS_DISTRO}-tf2-geometry-msgs" \
    "ros-${ROS_DISTRO}-derived-object-msgs" \
    "ros-${ROS_DISTRO}-vision-msgs" \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-image-geometry" \
    "ros-${ROS_DISTRO}-message-filters" \
    "ros-${ROS_DISTRO}-ackermann-msgs" \
    "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
    "ros-${ROS_DISTRO}-topic-tools" \
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

# ── Visualization and debug tools ────────────────────────────────────────────
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    # 3D visualizer
    "ros-${ROS_DISTRO}-rviz2" \
    # bag recording/playback + MCAP storage plugin
    "ros-${ROS_DISTRO}-rosbag2" \
    "ros-${ROS_DISTRO}-rosbag2-storage-mcap" \
    # rqt plugin suite: graph, plot, image-view, console, topic, service-caller
    "ros-${ROS_DISTRO}-rqt-common-plugins" \
    "ros-${ROS_DISTRO}-rqt-image-view" \
    # time-series plotter
    "ros-${ROS_DISTRO}-plotjuggler-ros" \
    # compressed/theora image transport (for camera visualization over the network)
    "ros-${ROS_DISTRO}-image-transport-plugins" \
    # keyboard teleoperation fallback
    "ros-${ROS_DISTRO}-teleop-twist-keyboard" \
    && sudo rm -rf /var/lib/apt/lists/*

# ── ROS2 workspace ───────────────────────────────────────────────────────────
ENV BUILD_HOME=/home/${USERNAME}/carla_ros_ws
RUN mkdir -p "${BUILD_HOME}/src"

# Copy custom.repos into workspace and import via vcstool
COPY custom.repos "${BUILD_HOME}/custom.repos"
RUN cd "${BUILD_HOME}" \
    && vcs import src < custom.repos

# Clone teleop/mux packages not in custom.repos. The CARLA ros-bridge is NOT
# cloned here — carla_ackermann_control / carla_twist_to_control run in the
# carla-ros-bridge container instead (see simulation_bringup.launch.py).
RUN cd "${BUILD_HOME}/src" \
    && git clone https://github.com/f1tenth/teleop_tools.git -b foxy-devel \
    && git clone https://github.com/f1tenth/ackermann_mux.git -b foxy-devel \
    && git clone https://github.com/ros-teleop/twist_mux.git -b rolling

# Copy the local autonomous_driving_simulators package from the build context
# instead of cloning it from GitHub, so local edits are baked into the image.
# Explicit per-directory copies preserve the package layout and keep build-cache
# invalidation scoped to the package (not the whole repo / compose files).
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

ARG SKIPPED_ROSDEP_KEYS="k4a lio_sam fast_lio rf2o_laser_odometry cmake_modules pacmod3_msgs python3-open3d open3d librealsense2 realsense2_camera libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"

RUN cd "${BUILD_HOME}" \
    && sudo apt-get update \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y -q \
       --os=ubuntu:jammy --rosdistro="${ROS_DISTRO}" \
       --skip-keys "${SKIPPED_ROSDEP_KEYS}" \
    && sudo rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/home/${USERNAME}/.ccache,uid=${USER_UID},gid=${USER_UID} \
    source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd "${BUILD_HOME}" \
    && colcon build --symlink-install \
       --event-handlers console_direct+ \
       --base-paths src \
       --cmake-args \
         ' -Wno-dev' ' --no-warn-unused-cli' \
         ' -DCMAKE_BUILD_TYPE=Release' \
         ' -DCMAKE_C_COMPILER_LAUNCHER=ccache' \
         ' -DCMAKE_CXX_COMPILER_LAUNCHER=ccache' \
         ' -DCMAKE_CUDA_COMPILER_LAUNCHER=ccache' \
         ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
         ' -DBOOST_BIND_GLOBAL_PLACEHOLDERS=1' \
         ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
         ' -DDOWNLOAD_ARTIFACTS=ON' \
    && ccache --show-stats

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ${BUILD_HOME}/install/setup.bash" >> ~/.bashrc \
    && echo "sudo sysctl -w net.core.rmem_max=2147483647" >> ~/.bashrc \
    && echo "sudo sysctl -w net.ipv4.ipfrag_time=3" >> ~/.bashrc \
    && echo "sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728" >> ~/.bashrc \
    && echo "sudo ip link set lo multicast on" >> ~/.bashrc

WORKDIR /home/${USERNAME}
