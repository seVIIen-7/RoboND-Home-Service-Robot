#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

bool checkMarkerStatus = false;

int main( int argc, char** argv )
 {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  
  // Create Publisher
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  // The lifetime field specifies how long this marker should stick around before being automatically deleted. A value of ros::Duration() means never to auto-delete
  marker.lifetime = ros::Duration();
  
  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1) // No subscriber that currently connected to the publisher
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  ROS_INFO("Connected to Marker Subscriber");

  marker_pub.publish(marker);

  // Set the marker action. Options are ADD, DELETE
  marker.action = visualization_msgs::Marker::ADD;

  // Publish the marker at the pick_up zone
  marker_pub.publish(marker);
  ROS_INFO("Successfully published marker at pick_up point");
  bool checkMarkerStatus = true;

  
  marker_pub.publish(marker);
  
//   std::string robot_position;
//   ros::param::get("/robot_position", robot_position);
  
  checkMarkerStatus = true;
  ros::Duration(5.0).sleep();
  while (ros::ok())
  {
//     ROS_INFO("test2");
    if (checkMarkerStatus) //pick up
    {
//       ROS_INFO("Robot Position %s", robot_position.c_str());
      marker.action = visualization_msgs::Marker::DELETE;
      // Publish the marker
      marker_pub.publish(marker);
      // Sleep for 5 sec
      checkMarkerStatus = false;
      ROS_INFO("Removed marker from pick_up point");
    }
    if (!checkMarkerStatus)
    {
      // Set the drop-off pose of the marker.
      // This is relative to the frame/time specified above
      ros::Duration(5.0).sleep();
      marker.pose.position.x = 1.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;

      // Set the marker action to ADD.
      marker.action = visualization_msgs::Marker::ADD;
      // Publish the marker
      marker_pub.publish(marker);
      ROS_INFO("Successfully published marker at drop_off point");
      // Sleep for 5 sec
      ros::Duration(5.0).sleep();
      }
    }
  r.sleep();
}


