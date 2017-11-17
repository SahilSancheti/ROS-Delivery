ssh turtlebot@red.local

# for mapping
roslaunch turtlebot_bringup minimal.launch --screen 
roslaunch turtlebot_navigation gmapping_demo.launch
# on local machine:
roslaunch turtlebot_rviz_launchers view_navigation.launch
roslaunch turtlebot_teleop keyboard_teleop.launch


# for autonomous driving
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/tmp/my_map.yaml
# on local machine:
roslaunch turtlebot_rviz_launchers view_navigation.launch --screen