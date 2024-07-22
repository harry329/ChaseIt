#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//TODO: Include the ball_chaser "DriveToTarget" header file

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_safe_move_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{

    ROS_INFO("DriveToTarget received - j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);
    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities, forward [0.5, 0.0]
    motor_command.linear.x = req.joint_1;
    motor_command.angular.z = req.joint_2;
    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);
    // Wait 3 seconds for arm to settle
    ros::Duration(3).sleep();
    // Return a response message
    res.msg_feedback = "Joint angles set - j1: " + std::to_string(req.joint_1) + " , j2: " + std::to_string(req.joint_2);
    ROS_INFO_STREAM(res.msg_feedback);
    return true;
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    // Create a ROS NodeHandle object
    ros::NodeHandle n;
    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    // Define a safe_move service with a handle_safe_move_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send joint commands");
    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}
