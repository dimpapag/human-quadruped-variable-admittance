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
#include <iomanip>
#include <ctime>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include "adm_controller.h"
#include <std_msgs/Float32MultiArray.h>

using namespace UNITREE_LEGGED_SDK;
using namespace Eigen;

ros::Publisher velocity_publisher;
double dt = 0.002; // Time step based on loop rate (500 Hz)
double t = 0.0; // Initial time
Vector2d vb = Vector2d::Zero(); // Initial velocity (x, y)
double wb = 0.0; // Yaw speed
Vector3d Fext = Vector3d::Zero(); // Initial external force
Vector3d pose = Vector3d::Zero(); // Initial external force


Matrix3d M_d = Matrix3d::Zero();
Matrix3d M_d_inv = Matrix3d::Zero();
Matrix3d D_d = Matrix3d::Zero();
Matrix3d A = Matrix3d::Zero();
Matrix3d B = Matrix3d::Zero();
Vector3d v_ad = Vector3d::Zero();
Vector3d v_ad_dot = Vector3d::Zero();

Matrix2d Rz = Matrix2d::Zero();
double power = 0.0;
double theta = 0.0;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Handle pose update if needed
}

void forceCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() == 3) {
        // Rename and assign the received force values
        float f_x = msg->data[0];
        float f_y = msg->data[1];
        float tau_z = msg->data[2];
        // Assign to Fext
        Fext(0) = f_x;
        Fext(1) = f_y;
        Fext(2) = tau_z;

        
    }
}

void initializing_robot(unitree_legged_msgs::HighCmd &high_cmd_ros)
{
    // Initial robot configuration
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.levelFlag = HIGHLEVEL;
    high_cmd_ros.mode = 2; 
    high_cmd_ros.gaitType = 1;
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
}

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg) {
    // Handle high state update if needed
    
}

std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&timestamp), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

// Define a global ofstream object for logging
std::ofstream info_log;

int main(int argc, char** argv) {
    ros::init(argc, argv, "admit_controller");

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
          << "Make sure the robot is standing on the ground." << std::endl
          << "\033[1mDo you want to save data for this run? (Y/N)\033[0m" << std::endl
          << "Enter 'Y' to save data or any other key to continue without saving..." << std::endl;

    char save_data_input;
    std::cin >> save_data_input;
    std::cin.ignore(); // Ignore newline character left in the input buffer

    bool enable_logging = (save_data_input == 'y' || save_data_input == 'Y');
    if (enable_logging) {
        std::cout << "\033[1;32mData logging enabled.\033[0m" << std::endl;
    } else {
        std::cout << "Data logging disabled. Continuing without saving." << std::endl;
    }
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);
    ros::Subscriber pose_subscriber = nh.subscribe("/odom", 1000, poseCallback);
    ros::Subscriber force_subscriber = nh.subscribe("/external_force", 10, forceCallback);
    // Subscribe to high state topic
    ros::Subscriber high_state_sub = nh.subscribe<unitree_legged_msgs::HighState>(
        "/high_state", 1, highStateCallback
    );
    unitree_legged_msgs::HighCmd high_cmd_ros;

    if (enable_logging) {
        // Create directory if it does not exist
        std::string directory = "/home/con/catkin_ws/src/command_give/Admittance_Logging_with_Keyboard";
        struct stat info;
        if (stat(directory.c_str(), &info) != 0) {
            // Directory does not exist, create it
            if (mkdir(directory.c_str(), 0777) == -1) {
                std::cerr << "Error: Failed to create directory " << directory << std::endl;
                return 1;
            }
        }

        // Create log file
        std::string filename = directory + "/adm_csv_" + getCurrentTimestamp() + ".csv";
        info_log.open(filename);
        if (!info_log.is_open()) {
            std::cerr << "Error: Failed to create log file " << filename << std::endl;
            return 1;
        }

        // Write header to log file
        info_log << "time(sec),v_x(m/s),v_y(m/s),omega_z(rad/s),f_x(N),f_y(N),tau_z(Nm),p_x(m),p_y(m),theta_z(rad),Power(Watt)" << std::endl;
    }

    ros::Rate loop_rate(500); // 500 Hz
    // Publish initial zero velocities
    initializing_robot(high_cmd_ros);

    // Desired Admittance Characteristics
    M_d(0, 0) = 10;
    M_d(1, 1) = 10;
    M_d(2, 2) = 2;

    D_d(0, 0) = 20;
    D_d(1, 1) = 20;
    D_d(2, 2) = 10;

    M_d_inv = M_d.inverse();
    A = -M_d_inv * D_d;
    B = M_d_inv;

    // Define saturation limits for vb and wb
    const double vb_max = 1.0;
    const double vb_min = -vb_max;
    const double wb_max = 3.52; 
    const double wb_min = -wb_max; 

    

    while (ros::ok() && t < 40.0) {
        // External force simulation
        // Vector3d Fext = Vector3d::Zero(); // No need to redeclare Fext here
        // Use the received force values
        // if ( t >= 3.0 && t <= 5.0) {
        //     Fext(0) = f_x;
        //     Fext(1) = f_y;
        //     Fext(2) = tau_z;
        // }
        

        // Admittance control calculation
        v_ad = v_ad + v_ad_dot * dt;
        v_ad_dot = A * v_ad + B * Fext;
        theta = theta + wb * dt;
        Rz <<  std::cos(theta), -std::sin(theta),  std::sin(theta), std::cos(theta);
        pose.head<2>() = pose.head<2>() + Rz * v_ad.head<2>() * dt;
        pose(2) = theta;

        // Extract velocity components
        vb = v_ad.head<2>(); // Extract first two rows
        wb = v_ad(2); // Extract the last element

        // Saturation of Linear Speed
        vb(0) = std::min(vb_max, std::max(vb_min, vb(0)));
        vb(1) = std::min(vb_max, std::max(vb_min, vb(1)));
        // Saturation of Rotational Speed
        wb = std::min(wb_max, std::max(wb_min, wb));

        //Other metrics :
        power =  v_ad.transpose()*Fext;



        std::cout << "Time: " << t << ", Velocity: (" << vb(0) << ", " << vb(1) << "), Yaw Speed: " << wb << ", Power: " << power << std::endl;

        // Publish velocity commands
        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.levelFlag = HIGHLEVEL;
        high_cmd_ros.mode = 2; 
        high_cmd_ros.gaitType = 1;
        high_cmd_ros.velocity[0] = vb(0); // X-axis velocity
        high_cmd_ros.velocity[1] = vb(1); // Y-axis velocity
        high_cmd_ros.yawSpeed = wb; // Yaw speed


        pub.publish(high_cmd_ros);
        // Log data if logging is enabled
        if (enable_logging) {
            info_log << std::fixed << std::setprecision(6) << t << "," << vb(0) << "," << vb(1) << "," << wb << "," << Fext(0) << "," << Fext(1) << "," << Fext(2) << "," << pose(0) << "," << pose(1) << "," << pose(2) << "," << power <<std::endl;
        }
        t += dt; // Increment time
        loop_rate.sleep(); // Control the loop rate
        ros::spinOnce();
    }
    high_cmd_ros.velocity[0] = 0.0; // X-axis velocity
    high_cmd_ros.velocity[1] = 0.0; // Y-axis velocity
    high_cmd_ros.yawSpeed = 0.0; // Yaw speed
    //For safety reasons ? : 
    //high_cmd_ros.mode = 0; 
    //high_cmd_ros.gaitType = 0;
    // Close log file if it's open
    if (info_log.is_open())
        info_log.close();


    return 0;
}
