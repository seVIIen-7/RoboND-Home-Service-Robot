#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){ // Wait for 5 seconds
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  // Update Robot Status with parameter server
  ros::param::set("/robot_position", "Start_Point");
  
  move_base_msgs::MoveBaseGoal goal_pick_up;
  move_base_msgs::MoveBaseGoal goal_drop_off;

  // set up the frame parameters
  goal_pick_up.target_pose.header.frame_id = "map";
  goal_pick_up.target_pose.header.stamp = ros::Time::now();
  
  goal_drop_off.target_pose.header.frame_id = "map";
  goal_drop_off.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to pick up, send a goal to move 1 meter forward
  goal_pick_up.target_pose.pose.position.x = 0.5;
  goal_pick_up.target_pose.pose.position.y = 0.5;
  goal_pick_up.target_pose.pose.position.y = 0.0;
  goal_pick_up.target_pose.pose.orientation.w = 1.0;

  // Define a position and orientation for the robot to drop off, send a goal to move 5 meter forward
  goal_drop_off.target_pose.pose.position.x = 1.0;
  goal_drop_off.target_pose.pose.position.y = 1.0;
  goal_drop_off.target_pose.pose.position.y = 0.0;
  goal_drop_off.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to pick up
  ROS_INFO("Sending pick up goal");
  ac.sendGoal(goal_pick_up);

  // Update Robot Status with paramter server
  ros::param::set("/robot_position", "Move_To_pick");
  
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot successfully moved to pick-up zone");
    ros::param::set("/robot_position", "Picked_Up");
  }

  else
  {
    ROS_INFO("Robot failed moving to pick-up zone");
    ros::param::set("/robot_position", "Failed_Pick");
  }


  // Pause 5 seconds after reaching the pickup zone, assuming the robot is picking up the stuffs
  ros::Duration(5.0).sleep();
  
   // Send the goal position and orientation for the robot to drop off
  ROS_INFO("Sending Drop Off Goal");
  ac.sendGoal(goal_drop_off);
  ros::param::set("/robot_position", "Move_To_Drop");

  // Wait an infinite time for the results until goal finishes
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot successfully moved to drop-off zone");
    ros::param::set("/robot_position", "Dropped_Off");
  }
  else
  {
    ROS_INFO("Robot failed moving to drop-off zone");
    ros::param::set("/robot_position", "Failed_Drop");
  }

  // Sleep for 5 seconds
  ros::Duration(5.0).sleep();
  
  return 0;
}
