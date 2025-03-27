#!/bin/bash

arg1=${1:-"display://0"}  #rtp://10.0.0.177:1234 | display://0

source /opt/ros/noetic/setup.bash
source /ros_deep_learning/devel/setup.bash

export ROS_MASTER_URI=http://wall-e:11311
export ROS_HOSTNAME=wallemain

#roslaunch ros_deep_learning video_viewer.ros1.launch input:=v4l2:///dev/video0 output:=display://0  

#roslaunch ros_deep_learning imagenet.ros1.launch input:=v4l2:///dev/video0 output:=rtp://10.0.0.50:1234

roslaunch ros_deep_learning detectnet.ros1.launch input:=v4l2:///dev/video0 output:="$arg1"

#roslaunch ros_deep_learning segnet.ros1.launch input:=v4l2:///dev/video0 output:=rtp://192.168.1.180:1234

