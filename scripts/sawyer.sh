#!/usr/bin/env bash

cd ~/ros_workspaces/ROS-Delivery/
./baxter.sh
rosrun intera_interface enable_robot.py -e
rosrun tf tf_echo base right_gripper
rosrun intera_interface joint_trajectory_action_server.py
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
rosrun sawyer pick_place.py