#!/bin/bash
#xhost +local:root
reset
mkdir -p /home/xuhao/output/output$1/
docker run --name swarm$1 --gpus all --rm -it \
    -v/home/xuhao/swarm_ws/:/root/swarm_ws/ \
    -v/home/xuhao/swarm_ws/:/home/xuhao/swarm_ws/ \
    -v/home/xuhao/source/:/home/xuhao/source/ \
    -v/home/xuhao/bags:/root/bags/ \
    -v/home/xuhao/output/output$1/:/root/output/ \
    --env="DISPLAY" \
    --privileged \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    xuhao1/swarm2020:pc /bin/zsh
