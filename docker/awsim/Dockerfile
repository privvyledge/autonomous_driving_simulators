# AWSIM Labs 1.1.1 — Unity-based simulator
# AWSIM publishes Autoware-compatible ROS2 topics natively; no ROS build needed.
ARG CUDA_VERSION=12.3.2
ARG CUDNN_VERSION=9
ARG UBUNTU_VERSION=22.04

FROM nvidia/cuda:${CUDA_VERSION}-cudnn${CUDNN_VERSION}-runtime-ubuntu${UBUNTU_VERSION}

ARG AWSIM_VERSION=1.1.1
ARG USERNAME=awsim_labs
ARG USER_UID=1000

ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_VISIBLE_DEVICES=all

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ=America/New_York
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -yq --no-install-recommends \
    sudo tzdata wget unzip \
    libvulkan1 libomp5 \
    xdg-user-dirs xdg-utils alsa-utils \
    libglib2.0-0 libxrender1 libxext6 libx11-6 \
    && ln -fns /usr/share/zoneinfo/${TZ} /etc/localtime \
    && dpkg-reconfigure -f noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --shell /bin/bash \
    -g root -G sudo,video "${USERNAME}" --uid "${USER_UID}" \
    && echo "${USERNAME}:${USERNAME}" | chpasswd \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

USER ${USERNAME}
WORKDIR /home/${USERNAME}

RUN mkdir -p Downloads && cd Downloads \
    && wget -q \
       "https://github.com/autowarefoundation/AWSIM-Labs/releases/download/v${AWSIM_VERSION}/awsim_labs_v${AWSIM_VERSION}.zip" \
    && unzip "awsim_labs_v${AWSIM_VERSION}.zip" \
    && rm "awsim_labs_v${AWSIM_VERSION}.zip" \
    && mv "awsim_labs_v${AWSIM_VERSION}" /home/${USERNAME}/awsim_simulator \
    && chmod +x /home/${USERNAME}/awsim_simulator/awsim_labs.x86_64

WORKDIR /home/${USERNAME}/awsim_simulator

# AWSIM requires a display — use DISPLAY env var or Xvfb on the host.
CMD ["./awsim_labs.x86_64"]
