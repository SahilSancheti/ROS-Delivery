#!/usr/bin/env bash

cd ~/ros_workspaces/ROS-Delivery/
./baxter.sh
rosrun intera_interface enable_robot.py -e
rosrun tf tf_echo base right_gripper
rosrun intera_interface joint_trajectory_action_server.py
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
roslaunch ar_track_alvar webcam_track.launch
rosrun rviz rviz

rosrun tf static_transform_publisher .045 0 -.07 0 -1.5708 0 ar_marker_14 base_marker 100
rosrun tf static_transform_publisher -.03 .01 .15 0 0 0 ar_marker_10 obj_marker 100
rosrun tf static_transform_publishe-.22 .01 .5 0 0 0 ar_marker_3 dest_marker 100
rosrun sawyer pick_place.py

#Check joint states
rostopic echo robot/joint_states

# Setup
#Tape AR Tag to base of robot arm
#Record ids of AR tag on base of robot arm, destination ids, and ids on the picked objects

# Status
#TODO: Relative position of new object to base of robot should be printed out and checked if same as tf echo right_gripper in same locaction, if not calibrate
#TODO: Avoid table and obstacles along the way by enforcing restrictions (may need to use rviz?? ask david)
#TODO: Remove hardcoded elements: base_id, dest_ids, obj_id