# CARLA 0.9.15 UE4 server — GPU only, no ROS
# Uses CUDA runtime (not devel) to keep image size down.
ARG CUDA_VERSION=12.3.2
ARG CUDNN_VERSION=9
ARG UBUNTU_VERSION=22.04

FROM nvidia/cuda:${CUDA_VERSION}-cudnn${CUDNN_VERSION}-runtime-ubuntu${UBUNTU_VERSION}

ARG CARLA_VERSION=0.9.15
ARG PYTHON_VERSION=3.10
ARG USERNAME=carla
ARG USER_UID=1000

ENV CARLA_VERSION=${CARLA_VERSION}
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_VISIBLE_DEVICES=all

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ=America/New_York
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -yq --no-install-recommends \
    sudo tzdata wget unzip \
    libvulkan1 libomp5 \
    xdg-user-dirs xdg-utils \
    alsa-utils \
    xvfb \
    python${PYTHON_VERSION} python3-pip \
    x11-apps \
    && ln -fns /usr/share/zoneinfo/${TZ} /etc/localtime \
    && echo "${TZ}" > /etc/timezone \
    && dpkg-reconfigure -f noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

# The nvidia-cuda base image does not ship the Vulkan ICD JSON that tells the
# loader where to find the NVIDIA Vulkan driver.  The actual libGLX_nvidia.so.0
# is injected at runtime by the NVIDIA Container Toolkit (NVIDIA_DRIVER_CAPABILITIES=all);
# we only need the pointer file.
RUN mkdir -p /usr/share/vulkan/icd.d \
    && printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libGLX_nvidia.so.0","api_version":"1.3.0"}}\n' \
       > /usr/share/vulkan/icd.d/nvidia_icd.json

RUN useradd --create-home --shell /bin/bash -g root -G sudo,video "${USERNAME}" \
    --uid "${USER_UID}" \
    && echo "${USERNAME}:${USERNAME}" | chpasswd \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Download and extract CARLA binary
RUN mkdir -p Downloads && cd Downloads \
    && wget -q "https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_${CARLA_VERSION}.tar.gz" \
    && mkdir -p /home/${USERNAME}/carla_simulator \
    && tar -xzf "CARLA_${CARLA_VERSION}.tar.gz" -C /home/${USERNAME}/carla_simulator \
    && rm "CARLA_${CARLA_VERSION}.tar.gz"

# Install CARLA Python client — used only by the healthcheck
RUN python3 -m pip install --no-cache-dir \
    "carla==${CARLA_VERSION}" 2>/dev/null \
    || python3 -m pip install --no-cache-dir \
       /home/${USERNAME}/carla_simulator/PythonAPI/carla/dist/carla-*.whl

RUN cd /home/${USERNAME}/carla_simulator \
    && /bin/bash ./ImportAssets.sh 2>/dev/null || true

ENV CARLA_ROOT=/home/${USERNAME}/carla_simulator

WORKDIR ${CARLA_ROOT}

# 2000: RPC   2001: streaming   2002: secondary ROS bridge port
EXPOSE 2000 2001 2002

HEALTHCHECK --interval=15s --timeout=10s --start-period=45s --retries=5 \
    CMD python3 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(5.0); c.get_server_version(); print('ok')"

CMD ["/bin/bash", "./CarlaUE4.sh", \
     "-prefernvidia", "-nosound", \
     "-benchmark", "-fps=20", \
     "-RenderOffScreen", "-quality-level=Low"]
