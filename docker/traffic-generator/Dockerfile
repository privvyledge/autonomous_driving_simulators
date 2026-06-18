# Traffic Generator — minimal Python image with CARLA client only.
# Runs generate_traffic.py to spawn vehicles and pedestrians in a live CARLA server.
FROM python:3.10-slim-bullseye

ARG CARLA_VERSION=0.9.14
ARG UBUNTU_VERSION=22
ARG PYTHON_MAJOR=3
ARG PYTHON_MINOR=10

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -yq --no-install-recommends wget \
    && rm -rf /var/lib/apt/lists/*

# CARLA Python client wheel from gezp's Ubuntu 22.04 / Python 3.10 build
RUN wget -q \
    "https://github.com/gezp/carla_ros/releases/download/carla-${CARLA_VERSION}-ubuntu-${UBUNTU_VERSION}.04/carla-${CARLA_VERSION}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-cp${PYTHON_MAJOR}${PYTHON_MINOR}-linux_x86_64.whl" \
    -O /tmp/carla.whl \
    && pip install --no-cache-dir /tmp/carla.whl \
    && rm /tmp/carla.whl

COPY scripts/generate_traffic.py /scripts/generate_traffic.py

ENV CARLA_HOST=localhost
ENV CARLA_PORT=2000

CMD ["python3", "/scripts/generate_traffic.py", \
     "--host", "localhost", "--port", "2000", \
     "--number-of-vehicles", "30", "--number-of-walkers", "10", \
     "--hybrid", "--safe", "--respawn", "--asynch"]
