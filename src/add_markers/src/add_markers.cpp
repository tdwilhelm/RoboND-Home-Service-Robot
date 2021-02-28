#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <math.h>

float x_odom = 0.0, y_odom = 0.0;

void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
  ::x_odom = msg->pose.pose.position.x;
  ::y_odom = msg->pose.pose.position.y;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(20);
  ros::Subscriber obom_sub = n.subscribe("/odom", 1000, odomCb);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CYLINDER;
  float champagnePos[3] = {-2.12969585732, -3.65482986593, -1.57};
  float executiveZone[3] = {-4.87520404477, -5.55359955952, -1.57};
  bool champagne = false;
  float x_distance, y_distance;
  float proximate = 0.2;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = champagnePos[0];
  marker.pose.position.y = champagnePos[1];
  marker.pose.position.z = 0.5;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(champagnePos[2]);


  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.1f;
  marker.color.b = 0.1f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  ROS_INFO("Champagne picked up!");

  while (ros::ok())
  {
      if (!champagne)
      {
        marker_pub.publish(marker);
        x_distance = fabs(marker.pose.position.x - x_odom);
        y_distance = fabs(marker.pose.position.y - y_odom);

        if( (x_distance < proximate) && (y_distance < proximate) )
        {
          marker.action = visualization_msgs::Marker::DELETE;
          marker_pub.publish(marker);
          ROS_INFO("Champagne dropped off!");
         champagne = true;
        }

      }
      else
      {
        x_distance = fabs(executiveZone[0] - x_odom);
        y_distance = fabs(executiveZone[1] - y_odom);

        if( (x_distance < proximate) && (y_distance < proximate) )
        {
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = executiveZone[0];
          marker.pose.position.y = executiveZone[1];
          marker.pose.position.z = 0.5;
          marker.pose.orientation = tf::createQuaternionMsgFromYaw(executiveZone[2]);
          marker_pub.publish(marker);          
        }
       }

    ros::spinOnce();
    r.sleep();

    }
    return 0;
}