source /opt/ros/noetic/setup.bash
source /home/walle/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://wall-e:11311
export ROS_HOSTNAME=wallemain

python /home/walle/catkin_ws/src/walle_files/walle_vision/src/talker.py
