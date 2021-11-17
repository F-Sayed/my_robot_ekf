#!/bin/sh
terminator -e "roslaunch my_robot new_test.launch" &
sleep 10
terminator -e "rosrun my_robot odom_pub" &
sleep 1
terminator -e "rosrun my_robot imu_pub.py" &
sleep 1
terminator -e "rosrun my_robot base_controller.py" &
sleep 1
terminator -e "rosrun imu_filter_madgwick imu_filter_node" 
