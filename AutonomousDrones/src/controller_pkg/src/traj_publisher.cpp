#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <nav_msgs/Path.h>
#include <sstream>
#include <iostream>

#include <math.h>

#define STATIC_POSE 1

#define PI M_PI

#define TFOUTPUT 1

// Trajectory message
trajectory_msgs::MultiDOFJointTrajectoryPoint msg;

// Quantities to fill in
tf::Transform desired_pose(tf::Transform::getIdentity());

// State trigger
int state_trigger = 0;

void nav_callback(nav_msgs::Path const& nav_goal){
    if (state_trigger == 2)
    {
        // receiving local trajectory from planer and 
        // set the 0.8 * total trajectory length as the goal point
        int length;
        length = trunc(nav_goal.poses.size() * 0.8);
        msg.transforms.resize(1);
        msg.transforms[0].translation.x = nav_goal.poses[length].pose.position.x;
        msg.transforms[0].translation.y = nav_goal.poses[length].pose.position.y;
        msg.transforms[0].translation.z = 2;        
        msg.transforms[0].rotation.x = nav_goal.poses[length].pose.orientation.x;
        msg.transforms[0].rotation.y = nav_goal.poses[length].pose.orientation.y;
        msg.transforms[0].rotation.z = nav_goal.poses[length].pose.orientation.z;
        msg.transforms[0].rotation.w = nav_goal.poses[length].pose.orientation.w;
    }
}

void cmd_vel_callback(geometry_msgs::Twist const& cmd){
    // set velocity
    geometry_msgs::Twist velocity;
    velocity.linear.x = cmd.linear.x;
    velocity.linear.y = cmd.linear.y;
    velocity.linear.z = cmd.linear.z;
    velocity.angular.x = cmd.angular.x;
    velocity.angular.y = cmd.angular.y;
    velocity.angular.z = cmd.angular.z;
    
    msg.velocities.resize(1);
    msg.velocities[0] = velocity;
}

void state_trigger_callback(const std_msgs::String::ConstPtr& trigger)
{
    if ((trigger->data.compare("Processing Scan")) == 0 || (trigger->data.compare("Processing Flying_back")) == 0)
        state_trigger = 2;
    else if((trigger->data.compare("Processing Take_off")) == 0)
    {
        state_trigger = 1;
        // Set take_off goal
        geometry_msgs::Twist origin_velocity;
        origin_velocity.linear.x = origin_velocity.linear.y = origin_velocity.linear.z = 0;
        origin_velocity.angular.x = origin_velocity.angular.y = origin_velocity.angular.z = 0;
        geometry_msgs::Twist origin_acceleration;
        origin_acceleration.linear.x = origin_acceleration.linear.y = origin_acceleration.linear.z = 0;
        origin_acceleration.angular.x = origin_acceleration.angular.y = origin_acceleration.angular.z = 0;
        tf::Vector3 origin(2, -1, 2);
        tf::Quaternion q_origin;
        q_origin.setRPY(0,0,0);
        desired_pose.setOrigin(origin);
        desired_pose.setRotation(q_origin);

        msg.transforms.resize(1);
        msg.transforms[0].translation.x = desired_pose.getOrigin().x();
        msg.transforms[0].translation.y = desired_pose.getOrigin().y();
        msg.transforms[0].translation.z = desired_pose.getOrigin().z();
        msg.transforms[0].rotation.x = desired_pose.getRotation().getX();
        msg.transforms[0].rotation.y = desired_pose.getRotation().getY();
        msg.transforms[0].rotation.z = desired_pose.getRotation().getZ();
        msg.transforms[0].rotation.w = desired_pose.getRotation().getW();
        msg.velocities.resize(1);
        msg.velocities[0] = origin_velocity;
        msg.accelerations.resize(1);
        msg.accelerations[0] = origin_acceleration;
    }
        
    else if((trigger->data.compare("Processing Landing")) == 0 || (trigger->data.compare("Finished")) == 0)
    {
        // Set landing goal
        geometry_msgs::Twist origin_velocity;
        origin_velocity.linear.x = origin_velocity.linear.y = origin_velocity.linear.z = 0;
        origin_velocity.angular.x = origin_velocity.angular.y = origin_velocity.angular.z = 0;
        geometry_msgs::Twist origin_acceleration;
        origin_acceleration.linear.x = origin_acceleration.linear.y = origin_acceleration.linear.z = 0;
        origin_acceleration.angular.x = origin_acceleration.angular.y = origin_acceleration.angular.z = 0;
        tf::Vector3 origin(2, -2, 0.16);
        tf::Quaternion q_origin;
        q_origin.setRPY(0,0,0);
        desired_pose.setOrigin(origin);
        desired_pose.setRotation(q_origin);

        msg.transforms.resize(1);
        msg.transforms[0].translation.x = desired_pose.getOrigin().x();
        msg.transforms[0].translation.y = desired_pose.getOrigin().y();
        msg.transforms[0].translation.z = desired_pose.getOrigin().z();
        msg.transforms[0].rotation.x = desired_pose.getRotation().getX();
        msg.transforms[0].rotation.y = desired_pose.getRotation().getY();
        msg.transforms[0].rotation.z = desired_pose.getRotation().getZ();
        msg.transforms[0].rotation.w = desired_pose.getRotation().getW();
        msg.velocities.resize(1);
        msg.velocities[0] = origin_velocity;
        msg.accelerations.resize(1);
        msg.accelerations[0] = origin_acceleration;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    ros::NodeHandle n;
    ros::Publisher desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/Quadrotor_1/desired_state", 1);
    ros::Subscriber nav_goal = n.subscribe("/Quadrotor_1/move_base/TrajectoryPlannerROS/local_plan", 1, nav_callback);
    ros::Subscriber cmd_vel = n.subscribe("/Quadrotor_1/cmd_vel", 1, cmd_vel_callback);
    ros::Subscriber state_trigger_sub = n.subscribe("/Quadrotor_1/state_trigger", 1, state_trigger_callback);
    ros::Rate loop_rate(500);
    ros::Time start(ros::Time::now());



    

#if TFOUTPUT
    tf::TransformBroadcaster br;
#endif

    int count = 0;
    while (ros::ok()) {

        if (state_trigger == 1 || state_trigger == 2)
            desired_state_pub.publish(msg);
        
        

        std::stringstream ss;
        ss << "Trajectory Position"
           << " x:" << desired_pose.getOrigin().x()
           << " y:" << desired_pose.getOrigin().y()
           << " z:" << desired_pose.getOrigin().z();
        ROS_INFO("%s", ss.str().c_str());       

#if TFOUTPUT
        br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
                                              "world", "av-desired"));
#endif

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
