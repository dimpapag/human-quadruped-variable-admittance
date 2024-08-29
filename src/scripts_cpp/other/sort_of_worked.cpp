#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Dense>
#include <iostream> // Include this for std::cout
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg) {
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile("/home/con/catkin_ws/src/unitree_ros/robots/go1_description/urdf/go1.urdf", tree)) {
        ROS_ERROR("Failed to construct kdl tree");
        return;
    }

    // Iterate through each leg
    for (const auto& leg : base_link) {
        KDL::Chain chain;
        KDL::Jacobian jacobian;

        // Get the segment from base to tip link
        if (!tree.getChain(leg.second, tip_link[leg.first], chain)) {
            ROS_ERROR_STREAM("Failed to get chain for leg " << leg.first);
            continue;
        }

        // Define joint positions
        KDL::JntArray q(chain.getNrOfJoints());
        for (int i = 0; i < 3; ++i) {
            q(i) = msg->motorState[i].q; // Assuming joint positions for each leg are stored contiguously
        }

        // Compute Jacobian
        KDL::ChainIkSolverVel_wdls solver(chain);
        solver.CartToJnt(q, jacobian);

        // Print Jacobian
        ROS_INFO_STREAM("Jacobian for leg " << leg.first << ":\n" << jacobian.data);
    }
}

int main(int argc, char** argv) {

    // Initialize ROS node
    ros::init(argc, argv, "force_measure");

    ros::NodeHandle nh;
    
    ros::Rate loop_rate(500);

    long motiontime = 0;
    unitree_legged_msgs::HighCmd high_cmd_ros;

    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);

    std::cout << "Reading URDF" << std::endl;
    // Load URDF model
    urdf::Model model;
    if (!model.initFile("/home/con/catkin_ws/src/unitree_ros/robots/go1_description/urdf/go1.urdf")) {
        ROS_ERROR("Failed to load URDF file");
        return -1;
    }
    std::cout << "URDF READ" << std::endl;

    // Create KDL tree from URDF
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
        ROS_ERROR("Failed to construct KDL tree from URDF");
        return -1;
    }
    
    // Define the base link
    std::string base_link = "base";

    // Define the end effectors (tip links) for each leg
    std::vector<std::string> tip_links = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};

    // Initialize total Jacobian matrix
    Eigen::MatrixXd total_jacobian(6, 12);
    total_jacobian.setZero();

    // Loop through each leg and compute the Jacobian matrix
    unsigned int column_offset = 0;
    for (const auto& tip_link : tip_links) {
        // Extract chain from base to tip link
        KDL::Chain chain;
        if (!tree.getChain(base_link, tip_link, chain)) {
            ROS_ERROR("Failed to extract chain from KDL tree");
            continue;
        }

        // Create KDL solver for the chain
        KDL::ChainJntToJacSolver solver(chain);

        // Assuming you have joint positions
        KDL::JntArray joint_positions(chain.getNrOfJoints());
        // Set joint positions here

        // Compute Jacobian matrix
        KDL::Jacobian kdl_jacobian;
        solver.JntToJac(joint_positions, kdl_jacobian);

        // Convert KDL Jacobian to Eigen matrix and store it in the total Jacobian
        for (unsigned int i = 0; i < kdl_jacobian.columns(); ++i) {
            for (unsigned int j = 0; j < kdl_jacobian.rows(); ++j) {
                total_jacobian(j, column_offset + i) = kdl_jacobian(j, i);
            }
        }

        // Update the column offset for the next leg
        column_offset += kdl_jacobian.columns();

        // Print KDL Jacobian data for this leg
        ROS_INFO_STREAM("KDL Jacobian Matrix for " << tip_link << ":\n" << kdl_jacobian.data);

        // Convert KDL Jacobian to Eigen matrix and print it
        Eigen::MatrixXd eigen_jacobian = Eigen::MatrixXd::Zero(6, chain.getNrOfJoints());
        for (unsigned int i = 0; i < kdl_jacobian.columns(); ++i) {
            for (unsigned int j = 0; j < kdl_jacobian.rows(); ++j) {
                eigen_jacobian(j, i) = kdl_jacobian(j, i);
            }
        }

        // Print Eigen Jacobian matrix for this leg
        ROS_INFO_STREAM("Eigen Jacobian Matrix for " << tip_link << ":\n" << eigen_jacobian);
    }

    // Print the total Jacobian matrix
    ROS_INFO_STREAM("Total Jacobian Matrix:\n" << total_jacobian);

        while (ros::ok())
    {

        motiontime += 2;

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

        pub.publish(high_cmd_ros);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
