#include <cmath>
#include <vector>
#include <array>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

//
namespace {
    const auto duration_seconds = 5.0;

    // TODO: Code duplication ...
    const std::vector<std::array<float, 3>> subgoals = {
            {-2.12969585732, -3.65482986593, -M_PI},
            {-4.87520404477, -5.55359955952, -1.55}
    };
}
void get_current_pose(const nav_msgs::Odometry::ConstPtr& msg)
{
  //pose[0] = msg->pose.pose.position.x;
  //pose[1] = msg->pose.pose.position.y;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "add_markers_test");
    ros::NodeHandle n;
    auto marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber pose_sub = n.subscribe("map", 10, get_current_pose);
    // Ensure we have subscribers for the marker.
    while (marker_pub.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the pose of the marker.
    marker.pose.position.z = 0.25;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    auto marker_taken = false;
    auto marker_created = false;

    while (ros::ok()) {
        if (!marker_created) {
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = subgoals[0][0];
            marker.pose.position.y = subgoals[0][1];
            marker_pub.publish(marker);

            ROS_INFO("Showing marker at source.");
            marker_created = true;
        }
        else if (!marker_taken) {
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);

            ROS_INFO("Hiding marker.");
            marker_taken = true;
        }
        else {
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = subgoals[1][0];
            marker.pose.position.y = subgoals[1][1];
            marker_pub.publish(marker);

            ROS_INFO("Showing marker at target.");
            break;
        }

        ros::spinOnce();
        ros::Duration(duration_seconds).sleep();
    }
    return 0;
}