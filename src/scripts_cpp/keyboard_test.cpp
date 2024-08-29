#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

// Callback function to process incoming force data
void forceCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // Extracting force components from the message data
    float f_x = msg->data[0];
    float f_y = msg->data[1];
    float f_z = msg->data[2];

    // Logging the received force values
    ROS_INFO("Received forces: F_x = %.2f, F_y = %.2f, F_z = %.2f", f_x, f_y, f_z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "force_receiver");  // Initialize the ROS node
    ros::NodeHandle nh;  // Create a NodeHandle

    // Subscribe to the "external_force" topic
    ros::Subscriber force_sub = nh.subscribe("external_force", 10, forceCallback);

    ros::spin();  // Enter the ROS event loop

    return 0;
}
