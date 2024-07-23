# Carla + ROS + Autoware Setup Instructions
## Requirements
### Hardware
* CPU: 64-bit Quad-core Intel or AMD CPU, 2.5 GHz or faster
* GPU: Nvidia Geforce 3070 and above with >= 6GB VRAM
* RAM: Minimum 8GB
* Disk Space: 20 GB (Carla) + 

### Software
* Carla: 
* rosdistro=humble
rmw_implementation=rmw_cyclonedds_cpp
base_image=ros:humble-ros-base-jammy
cuda_version=12.3
cudnn_version=8.9.5.29-1+cuda12.2
tensorrt_version=8.6.1.6-1+cuda12.0

## Installation
### Docker 


echo "
if [ ! -e /tmp/cycloneDDS_configured ]; then
    sudo sysctl -w net.core.rmem_max=2147483647
    sudo sysctl -w net.ipv4.ipfrag_time=3
    sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728     # (128 MB)
    sudo ip link set lo multicast on
    touch /tmp/cycloneDDS_configured
fi
" >> ~/.bashrc