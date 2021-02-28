#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(8.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  float champagnePos[3] = {-2.12969585732, -3.75482986593, -1.57};
  float executiveZone[3] = {-4.87520404477, -5.55359955952, -1.57};

  move_base_msgs::MoveBaseGoal goal;
	
  // set up the frame parameters
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = champagnePos[0];
  goal.target_pose.pose.position.y = champagnePos[1];	
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(champagnePos[1]);

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Moving towards employee bar warehouse!");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("champagne has been picked up!");
    
    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = executiveZone[0];
    goal.target_pose.pose.position.y = executiveZone[1];	
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(executiveZone[1]);

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Moving towards Executive bar!");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("champagne arrived at CEO bar for safe keeping!");      
    sleep(5);
    return 0;
    } else {
      ROS_INFO("The robot can't move to CEO office bar!");
      return 0;
    }		
  } else 
    ROS_INFO("The robot can't reach the CEO office bar!");

  return 0;
}