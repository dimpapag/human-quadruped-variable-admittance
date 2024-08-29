#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <nav_msgs/Odometry.h>
#include "include/convert.h"
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <iostream>
#include <ctime>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>

using namespace UNITREE_LEGGED_SDK;
using namespace Eigen;

ros::Publisher velocity_publisher;
double dt = 0.002; // Time step based on loop rate (500 Hz)
double t = 0.0; // Initial time
Vector2d vb = Vector2d::Zero(); // Initial velocity (x, y)
double wb = 0.0; // Yaw speed
Vector3d Fext = Vector3d::Zero(); // Initial external force

Matrix3d M_d = Matrix3d::Zero();
Matrix3d D_d = Matrix3d::Zero();
Matrix3d A = Matrix3d::Zero();
Matrix3d B = Matrix3d::Zero();
Vector3d v_ad = Vector3d::Zero();
Vector3d v_ad_dot = Vector3d::Zero();

std::vector<double> log_t = {0};
std::vector<double> log_vb = {0};
std::vector<double> log_wb = {0};
Vector3d log_Fext = Vector3d::Zero();

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Handle pose update if needed
}

std::string getTimestamp() {
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);
    std::stringstream ss;
    ss << (now_tm->tm_year + 1900)
       << (now_tm->tm_mon + 1)
       << now_tm->tm_mday
       << "_"
       << now_tm->tm_hour
       << now_tm->tm_min
       << now_tm->tm_sec;
    return ss.str();
}

void saveDataToCsv() {
    std::string folderPath = "/home/con/catkin_ws/src/command_give/Admittance_Logging";
    // Create directory if it doesn't exist
    struct stat info;
    if(stat(folderPath.c_str(), &info) != 0) {
        std::string command = "mkdir -p " + folderPath; // -p flag ensures the creation of parent directories if they don't exist
        system(command.c_str());
    }

    std::ofstream file;
    std::string timestamp = getTimestamp();
    std::string filename = folderPath + "/trajectory_log_" + timestamp + ".csv";
    file.open(filename);

    file << "Time,Velocity_x,Velocity_y,Yaw_speed,Fext_x,Fext_y,Fext_z\n";
    for (size_t i = 0; i < log_t.size(); ++i) {
        file << log_t[i] << "," << vb(0) << "," << vb(1) << "," << wb << "," 
             << log_Fext(0) << "," << log_Fext(1) << "," << log_Fext(2) << "\n";
    }

    file.close();
    ROS_INFO("Data saved to %s", filename.c_str());
}

void publishZeroVelocities() {
    unitree_legged_msgs::HighCmd high_cmd_ros;
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.levelFlag = HIGHLEVEL;
    high_cmd_ros.mode = 0;
    high_cmd_ros.gaitType = 0;
    high_cmd_ros.speedLevel = 0;
    high_cmd_ros.footRaiseHeight = 0;
    high_cmd_ros.bodyHeight = 0;
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.velocity[0] = 0.0f;
    high_cmd_ros.velocity[1] = 0.0f;
    high_cmd_ros.yawSpeed = 0.0f;
    high_cmd_ros.reserve = 0;
    velocity_publisher.publish(high_cmd_ros);
}

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg) {
    // Handle high state update if needed
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "admit_controller");

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Execute -Admittance Controller- ?" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore(); // Wait for user input before proceeding

    ros::NodeHandle nh;

    velocity_publisher = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);
    ros::Subscriber pose_subscriber = nh.subscribe("/odom", 1000, poseCallback);
    // Subscribe to high state topic
    ros::Subscriber high_state_sub = nh.subscribe<unitree_legged_msgs::HighState>(
        "/high_state", 1, highStateCallback
    );
    ros::Rate loop_rate(500); // 500 Hz

    // Desired Admittance Characteristics
    M_d(0, 0) = 10;
    M_d(1, 1) = 10;
    M_d(2, 2) = 2;

    D_d(0, 0) = 40;
    D_d(1, 1) = 40;
    D_d(2, 2) = 20;

    //M_d_inv = M_d.inverse();
    A = -M_d.inverse() * D_d;
    B = M_d.inverse();
    // Publish initial zero velocities
    publishZeroVelocities();

    while (ros::ok()&& t<10.0) {
        // External force simulation
        Vector3d Fext = Vector3d::Zero();
        if ( t>=3.0 && t<=5.0) {
            Fext(0) = 0.0;
            Fext(1) = 10.0;
        }
        
        // Admittance control calculation
        v_ad = v_ad + v_ad_dot * dt;//Euler integration
        v_ad_dot = A * v_ad + B * Fext;

        // Extract velocity components
        vb = v_ad.head<2>(); // Extract first two rows
        wb = v_ad(2); // Extract the last element

        // Log data
        log_t.push_back(t);
        log_vb.push_back(vb.norm());
        log_wb.push_back(wb);
        log_Fext = Fext;

        // Publish velocity commands
        unitree_legged_msgs::HighCmd high_cmd_ros;
        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.levelFlag = HIGHLEVEL;
        high_cmd_ros.mode = 2;
        high_cmd_ros.gaitType = 1; // trot
        high_cmd_ros.velocity[0] = vb(0); // X-axis velocity
        high_cmd_ros.velocity[1] = vb(1); // Y-axis velocity
        high_cmd_ros.yawSpeed = wb; // Yaw speed
        high_cmd_ros.footRaiseHeight = 0.1;

        velocity_publisher.publish(high_cmd_ros);
        
        t += dt; // Increment time
        loop_rate.sleep(); // Control the loop rate
    }
    high_cmd_ros.mode = 1;
    saveDataToCsv();

    return 0;
}