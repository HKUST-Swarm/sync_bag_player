#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/xuhao/d2slam_ws/devel/setup.bash
echo "Run bag replay for drone",$1, "with bag", $2
export DRONE_ID=$1
route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0
chmod a+x /root/entry_point.sh
# /root/entry_point.sh 2>&1 1>&1 | tee /root/output/logs.txt &
rosrun sync_bag_player sync_rosplay.py --path $2  --drone-id $1 --config-path /root/config.yaml --token $3 &
# /root/entry_point.sh 2>&1 | tee /root/output/logs.txt
/root/entry_point.sh
echo "Will end bag record"
pkill -TERM record
sleep 10000