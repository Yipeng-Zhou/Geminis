#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <sstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void state_trigger_callback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("Processing Flying_back") == 0) // waiting for the back trigger from state machine
    {
      // tell the action client that we want to spin a thread by default
      MoveBaseClient ac("/Quadrotor_1/move_base", true);

      // wait for the action server to come up
      while(!ac.waitForServer(ros::Duration(5.0))){
          ROS_INFO("Waiting for the move_base action server to come up");
      }

      move_base_msgs::MoveBaseGoal goal;

      // set the back goal for the drone 1
      goal.target_pose.header.frame_id = "world";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = 2.0;
      goal.target_pose.pose.position.y = -2.0;
      goal.target_pose.pose.position.z = 2;
      goal.target_pose.pose.orientation.w = 1.0;

      ROS_INFO("Sending goal");

      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Quadrotor1 has arrived!");
        // sometimes it will not get a succeeded information in this part. So we don't use it here
      }
      else
          ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
    else
    {
      ROS_INFO("Waiting for back trigger");
    }
        
}

int main(int argc, char** argv){
  ros::init(argc, argv, "back_navigation_1");
  // initialize the node handel
  ros::NodeHandle nh;
  // subscribe from /state_trigger
  ros::Subscriber state_trigger_sub = nh.subscribe("/Quadrotor_1/state_trigger", 1, state_trigger_callback);
  ros::spin();
  return 0;
}