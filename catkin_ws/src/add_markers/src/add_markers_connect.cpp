#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

class AddMarker 
{

private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber odom_sub;
  visualization_msgs::Marker marker;
  double pickupZone[2] = {0.0, 0.0};
  double dropoffZone[2] = {0.0, 1.0};
  double threshold;
  const int threshold_multiplier = 3;

public:
  AddMarker() 
  {
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    odom_sub = n.subscribe("/robot_position", 1, &AddMarker::odomCallback, this);

    // initialize marker and show marker at pickup zone
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers_connect";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    // Set marker's initial coordinates
    setMarkerPosition(pickupZone[0], pickupZone[1]);

    marker.lifetime = ros::Duration();

    threshold = marker.scale.x;

    publishMarker();
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) 
  {
    float pos_x = msg->pose.pose.position.x;
    float pos_y = msg->pose.pose.position.y;

    // give position tolerance of + or - the marker's radius
    bool atPickupZone = (pickupZone[0] - threshold < pos_x && pos_x < pickupZone[0] + threshold)
      && (pickupZone[1] - threshold < pos_y && pos_y < pickupZone[1] + threshold);
    bool atDropoffZone = (dropoffZone[0] - threshold * threshold_multiplier <= pos_x && pos_x <= dropoffZone[0] + threshold * threshold_multiplier)
      && (dropoffZone[1] + threshold < pos_y);          
    // pause 5s to simulate pickup
    ros::Duration(5).sleep();
    // hide marker and set it to new coordinates
    setMarkerPosition(dropoffZone[0], dropoffZone[1]);
    marker.color.a = 0.0;

    if (atPickupZone) {
      // pause 5s to simulate pickup
      ros::Duration(5).sleep();
      // hide marker  marker.color.a = 1.0;
      ROS_INFO("Successfully arrived pick_up zone");
    }

    // publish marker
    publishMarker();

  }

  void setMarkerPosition(double pos_x, double pos_y) 
  {
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pos_x;
    marker.pose.position.y = pos_y;
  }

  void publishMarker() 
  {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) 
    {
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
  }
  
};

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "add_markers_connect");

  AddMarker addMarker;
  
  ros::spin();

  return 0;
  
}


