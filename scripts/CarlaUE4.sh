#!/bin/sh
kill -9 $ps aux | grep Unreal;
#export CARLA_ROOT=/home/${USERNAME}/carla_simulator
chmod +x "$CARLA_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
"$CARLA_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" CarlaUE4 "$@"
#"$CARLA_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" CarlaUE4 -nosound -prefernvidia -benchmark -fps=30