# Home Service Robot
In this project, a full home service robot capable of navigating to pick up and deliver virtual objects.
The package looks like this:
```
    ├──                                # Official ROS packages
    |
    ├── slam_gmapping                  # gmapping_demo.launch file                   
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file      
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file 
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├──                                # Packages and direcotries built
    |
    ├── map                          # map files
    │   ├── ...
    ├── scripts                   # shell scripts files
    │   ├── ...
    ├──rvizConfig                      # rviz configuration files
    │   ├── ...
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    └──
 ```
 ## Testing
 ### SLAM Testing
 This part is to manually test SLAM. It will deploy a turtlebot inside the environment.
 ```
 $ cd catkin_ws/src/scripts
 $ ./test_slam.sh
 ```
 In `xterminal` of teleop, use keyboards command to mannually control the robot and visualize the map in RViz.
 ### Localization and Navigation Testing
 This section is only for testing purposes to make sure the robot is able to reach these positions before   autonomously commanding it to travel towards them. The ROS Navigation stack based on the **Dijkstra's** is applied.
 ```
 $ cd catkin_ws/src/scripts
 $ ./test_navigation.sh
 ```
 Once you launch all the nodes, you will initially see the particles around your robot, which means that AMCL recognizes the initial robot pose. Now, manually point out to two different goals, one at a time, and direct your robot to reach them and orient itself with respect to them.
 
 ### Reaching Multiple Goals
 Here, the node `pick_objects` communicates with the ROS navigation stack and autonomously send successive goals for the robot to reach. The node is based on the [ROS tutorial](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals) that describes how to send a single goal position and orientation to the navigation stack.
 
 The first goal is the desired pickup goal and the second goal is the desired drop off goal. The robot travels to the desired pickup zone, displays a message that it reached its destination, waits 5 seconds, travels to the desired drop off zone, and displays a message that it reached the drop off zone.
 ```
 $ cd catkin_ws/src/scripts
 $ ./pick_objects.sh
 ```
 ![pick_objects](https://github.com/seVIIen-7/RoboND-Home-Service-Robot/blob/master/pick_objects.jpg)
 ### Modeling Virtual Objects
 Here, the node `pick_objects` draws markers in RViz. The node is based on the [ROS tutorial](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes) that describes how to publish a different shape each second at the same position and orientation. 
 
 This node follows this algorithm:
 - Publish the marker at the pickup zone
 - Pause 5 seconds
 - Hide the marker
 - Pause 5 seconds
 - Publish the marker at the drop off zone
 ```
 $ cd catkin_ws/src/scripts
 $ ./add_markers.sh
 ```
 In RViz, Add `Marker`
 ![add_objects](https://github.com/seVIIen-7/RoboND-Home-Service-Robot/blob/master/add_markers.jpg)
 ### Full Home Service Robot
 Here is a simulation of a full home service robot capable of navigating to pick up and deliver virtual objects. The `add_markers` and `pick_objects` nodes can communicate by subscribing to the **odometry** to keep track of the robot pose.
 
 This node follows this algorithm:
 - Publish the marker at the pickup zone
 - Pause 5 seconds
 - Hide the marker
 - Pause 5 seconds
 - Publish the marker at the drop off zone
 ```
 $ cd catkin_ws/src/scripts
 $ ./home_service.sh
 ```
 

