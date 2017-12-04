# ROS-Delivery
Uses Kobuki TurtleBot 2 Platform to deliver items to and from Sawyer in ROS
[![Alt text](https://img.youtube.com/vi/Up7FTVMkmsk/0.jpg)](https://www.youtube.com/watch?v=Up7FTVMkmsk)
# Running the TurtleBot

## SSH into the TurtleBot
    ssh turtlebot@red.local
## Export ROS_MASTER_URI in Workstation Terminals
    export ROS_MASTER_URI=http://yellow.local:11311

## Generate a map of the room.
### Run on the TurtleBot
    roslaunch turtlebot_bringup minimal.launch --screen 
    roslaunch turtlebot_navigation gmapping_demo.launch
### Run on the Workstation
    roslaunch turtlebot_rviz_launchers view_navigation.launch
    roslaunch turtlebot_teleop keyboard_teleop.launch

Drive the TurtleBot around until a satisfactory map is generated in RViz.

### Save the map
    rosrun map_server map_saver -f /tmp/my_map

## Drive Autonomously with Object Avoidance
### Run on the TurtleBot to load the map
    roslaunch turtlebot_bringup minimal.launch
    roslaunch turtlebot_navigation amcl_demo.launch map_file:=/tmp/my_map.yaml

### Run on the Workstation
    roslaunch turtlebot_rviz_launchers view_navigation.launch --screen

In RViz, select “2D Pose Estimate” and click the map approximately where TurtleBot is and align the arrow to indicate orientation.

### Run the Publisher and Subscriber to command the TurtleBot to move to user specified points.
    rosrun turtlebot sub.py
    rosrun turtlebot pub.py

