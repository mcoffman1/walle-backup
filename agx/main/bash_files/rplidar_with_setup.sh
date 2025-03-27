#!/usr/bin/env bash

export PYTHONPATH=/home/walle/.local/lib/python3.8/site-packages:$PYTHONPATH

source /opt/ros/noetic/setup.bash
source /home/walle/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://wall-e:11311
export ROS_HOSTNAME=wallemain

roslaunch rplidar_ros rplidar_a1.launch

