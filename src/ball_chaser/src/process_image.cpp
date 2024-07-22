#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.joint_1 = lin_x
    srv.joint_2 = ang_z
    if(!client.call(srv))
        ROS_ERROR("Failed to call service drive_bot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int white_pixel = 255;
    bool move_robot = false;
    bool white_pixel_found = false;
    int left_boundary = img.width / 3;
    int right_boundary = 2 * img.width / 3;

    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height * img.step; i += 3) {
        if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) {
            white_pixel_found = true;

            // Identify if this pixel falls in the left, mid, or right side of the image
            int pixel_position = (i % (img.width * img.step)) / 3;

            if (pixel_position < left_boundary) {
                // White pixel is in the left side of the image
                drive_bot(0.5, 1.0); // Example velocities, adjust as needed
            } else if (pixel_position < right_boundary) {
                // White pixel is in the middle of the image
                drive_bot(0.5, 0.0); // Example velocities, adjust as needed
            } else {
                // White pixel is in the right side of the image
                drive_bot(0.5, -1.0); // Example velocities, adjust as needed
            }

            move_robot = true;
            break; // Break the loop once a white pixel is found and action is taken
        }
    }

    // Request a stop when there's no white ball seen by the camera
    if (!white_pixel_found) {
        drive_bot(0.0, 0.0); // Stop the robot
    }

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}