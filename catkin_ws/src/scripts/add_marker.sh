#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/map/Myworld.world" &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/map/my_house.yaml" &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/workspace/catkin_ws/devel/setup.bash; rosrun add_markers add_markers" &
sleep 5
