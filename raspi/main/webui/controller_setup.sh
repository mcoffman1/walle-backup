## auto source ros
source /opt/ros/noetic/setup.bash
source /home/walle/catkin_ws/devel/setup.bash

#http://Wall-E:11311
export ROS_MASTER_URI=http://wall-e:11311
export ROS_HOSTNAME=wall-e

roslaunch navstack_pub teleop_joy.launch
