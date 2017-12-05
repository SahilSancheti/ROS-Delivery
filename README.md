# ROS-Delivery
Uses Kobuki TurtleBot 2 Platform to deliver items to and from Sawyer in ROS <br>
https://sites.google.com/view/ros-delivery/home
<br>

[![Alt text](https://github.com/SahilSancheti/ROS-Delivery/blob/master/scripts/video.png)](https://www.youtube.com/watch?v=Up7FTVMkmsk)

# Running the TurtleBot

#### SSH into the TurtleBot
    ssh turtlebot@red.local
#### Export ROS_MASTER_URI in Workstation Terminals
    export ROS_MASTER_URI=http://yellow.local:11311

### Generate a map of the room.
#### Run on the TurtleBot
    roslaunch turtlebot_bringup minimal.launch --screen 
    roslaunch turtlebot_navigation gmapping_demo.launch
#### Run on the Workstation
    roslaunch turtlebot_rviz_launchers view_navigation.launch
    roslaunch turtlebot_teleop keyboard_teleop.launch

Drive the TurtleBot around until a satisfactory map is generated in RViz.

#### Save the map
    rosrun map_server map_saver -f /tmp/my_map

### Drive Autonomously with Object Avoidance
#### Run on the TurtleBot to load the map
    roslaunch turtlebot_bringup minimal.launch
    roslaunch turtlebot_navigation amcl_demo.launch map_file:=/tmp/my_map.yaml

#### Run on the Workstation
    roslaunch turtlebot_rviz_launchers view_navigation.launch --screen

In RViz, select “2D Pose Estimate” and click the map approximately where TurtleBot is and align the arrow to indicate orientation.

#### Run the Publisher and Subscriber to command the TurtleBot to move to user specified points.
    rosrun turtlebot sub.py
    rosrun turtlebot pub.py

# Running the Sawyer Pick and Place

#### SSH into the Sawyer robot to run code on Sawyer
    ./baxter.sh

#### Run on Sawyer in different Terminal windows to initialize the robot
    rosrun intera_interface enable_robot.py -e
    rosrun intera_interface joint_trajectory_action_server.py
    roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true

#### Run on the Workstation in different Terminal window to enable the USB camera and AR tracking
    roslaunch ar_track_alvar webcam_track.launch
    rosrun rviz rviz
    
#### Run on Workstation to publish static transforms (may need to calibrate yourself)
    rosrun tf static_transform_publisher .045 0 -.07 0 -1.5708 0 ar_marker_14 base_marker 100
    rosrun tf static_transform_publisher -.01 .02 .15 0 0 0 ar_marker_6 obj_marker 100
    rosrun tf static_transform_publisher -.08 0.0 .45 0 0 0 ar_marker_3 dest_marker 100

#### Run on Workstation to activate pick and place
    rosrun sawyer run.sh

#### (Optional) Echo joint states and gripper locations for debugging
    rosrun tf tf_echo base right_gripper
    rostopic echo robot/joint_states
