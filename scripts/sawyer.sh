#!/usr/bin/env bash

cd ~/ros_workspaces/ROS-Delivery/
./baxter.sh
rosrun intera_interface enable_robot.py -e
rosrun tf tf_echo base right_gripper
