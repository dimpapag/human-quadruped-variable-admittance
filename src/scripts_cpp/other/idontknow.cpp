#include <ros/ros.h>
#include <unitree_legged_msgs/HighState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Dense>
#include <iostream>

// Define vectors to store joint configurations and estimated torques for each leg
Eigen::VectorXd all_q(12), all_tauEst(12);
Eigen::Vector3d q_FR, q_FL, q_RR, q_RL;
Eigen::Vector3d tau_FR, tau_FL, tau_RR, tau_RL;

// Define callback function to handle high state messages
void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg)
{
    // Extract all joint configurations (q) and estimated joint torques (tauEst)
    // Assuming that `msg->motorState` contains motor states and `msg->motorState[i].tauEst` contains estimated torques

    for (int i = 0; i < 12; ++i) {
        all_q(i) = msg->motorState[i].q;
        all_tauEst(i) = msg->motorState[i].tauEst;
    }

    // Separate joint configurations and estimated torques for each leg
    q_FR = all_q.segment<3>(0);
    q_FL = all_q.segment<3>(3);
    q_RR = all_q.segment<3>(6);
    q_RL = all_q.segment<3>(9);

    tau_FR = all_tauEst.segment<3>(0);
    tau_FL = all_tauEst.segment<3>(3);
    tau_RR = all_tauEst.segment<3>(6);
    tau_RL = all_tauEst.segment<3>(9);
}

// Function to compute Jacobian and print results for a leg
void computeJacobianAndPrintResults(const Eigen::Vector3d& q, const Eigen::Vector3d& tau, const KDL::Chain& chain)
{
    // Compute Jacobian
    KDL::ChainJntToJacSolver jac_solver(chain);
    KDL::Jacobian jac(chain.getNrOfJoints());
    KDL::JntArray q_array(3);
    for (int i = 0; i < 3; ++i) {
        q_array(i) = q(i);
    }
    jac_solver.JntToJac(q_array, jac);

    // Compute inverse and transpose of the Jacobian
    Eigen::MatrixXd J_inv = jac.data.topLeftCorner(3, jac.columns()).transpose().inverse();

    // Print results
    std::cout << "T-Inverse of Jacobian matrix:" << std::endl;
    std::cout << J_inv << std::endl;
    std::cout << "Torque: " << tau.transpose() << std::endl;
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "your_node_name");
    ros::NodeHandle nh;

    // Subscribe to high state topic
    ros::Subscriber high_state_sub = nh.subscribe<unitree_legged_msgs::HighState>(
        "/high_state", 1, highStateCallback
    );

    // Define loop rate (adjust as needed)
    ros::Rate loop_rate(10); // 10 Hz

    // URDF file path
    std::string urdf_file = "/home/con/catkin_ws/src/unitree_ros/robots/go1_description/urdf/go1.urdf";

    // Initialize KDL variables
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_file, tree)) {
        std::cerr << "Failed to parse URDF file." << std::endl;
        return 1;
    }

    // Main loop
    while (ros::ok()) {
        ros::spinOnce(); // Process any incoming messages

        // Compute Jacobian and print results for each leg
        computeJacobianAndPrintResults(q_FR, tau_FR, tree.getChain("base", "FR_foot"));
        computeJacobianAndPrintResults(q_FL, tau_FL, tree.getChain("base", "FL_foot"));
        computeJacobianAndPrintResults(q_RR, tau_RR, tree.getChain("base", "RR_foot"));
        computeJacobianAndPrintResults(q_RL, tau_RL, tree.getChain("base", "RL_foot"));

        loop_rate.sleep(); // Control the loop rate
    }

    return 0;
}
