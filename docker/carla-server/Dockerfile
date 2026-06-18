# CARLA UE4 server — GPU only, no ROS
# Base: official carlasim image; ships the full UE4/Vulkan/GL runtime.
# Default 0.9.14 to match the rest of the stack; override with CARLA_VERSION.
ARG CARLA_VERSION=0.9.14

FROM carlasim/carla:${CARLA_VERSION}

ARG CARLA_VERSION=0.9.14

ENV CARLA_VERSION=${CARLA_VERSION}
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_VISIBLE_DEVICES=all

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ=America/New_York
ARG DEBIAN_FRONTEND=noninteractive

# Add xvfb (headless virtual display) and sudo.
# The official image runs as user 'carla'; we need root momentarily.
USER root

RUN apt-get update && apt-get install -yq --no-install-recommends \
    sudo xvfb \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /etc/sudoers.d \
    && echo "carla ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/carla \
    && chmod 0440 /etc/sudoers.d/carla \
    && mkdir -p /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix \
    && printf '%s\n' \
       '#!/bin/bash' \
       'set -e' \
       'CARLA_ARGS=(-prefernvidia -nosound -benchmark "-fps=${CARLA_FPS:-20}" "-quality-level=${CARLA_QUALITY:-Low}")' \
       'if [ "${CARLA_HEADLESS:-true}" = "true" ]; then' \
       '    sudo mkdir -p /tmp/.X11-unix && sudo chmod 1777 /tmp/.X11-unix' \
       '    # Clear stale X lock/socket left by a previous crashed run so Xvfb :99 can rebind' \
       '    sudo rm -f /tmp/.X99-lock /tmp/.X11-unix/X99' \
       '    Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp &' \
       '    until [ -S /tmp/.X11-unix/X99 ]; do sleep 0.2; done' \
       '    export DISPLAY=:99' \
       '    CARLA_ARGS+=(-RenderOffScreen)' \
       'fi' \
       'exec /bin/bash CarlaUE4.sh "${CARLA_ARGS[@]}"' \
       > /entrypoint.sh \
    && chmod +x /entrypoint.sh

USER carla

# 2000: RPC   2001: streaming   2002: secondary ROS bridge port
EXPOSE 2000 2001 2002

HEALTHCHECK --interval=15s --timeout=10s --start-period=45s --retries=5 \
    CMD python3 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(5.0); c.get_server_version(); print('ok')"

ENTRYPOINT ["/entrypoint.sh"]
