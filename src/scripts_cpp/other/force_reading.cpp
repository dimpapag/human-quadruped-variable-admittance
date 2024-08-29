#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <unitree_legged_msgs/MotorState.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include <vector>


using namespace UNITREE_LEGGED_SDK;


//DIKO MOU : GIA TA MOTOR STATES (Q,Q_DOT,TAU)
    //unitree_legged_msgs::MotorState motorState[20];

//    void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg) {
    // Extract tauEst values from the received message
//    VectorXf tauEst(20);
 //   for (int i = 0; i < 20; ++i) {
 //       tauEst(i) = msg->motorState[i].tauEst;
 //   }

    // Calculate Jacobian matrix (example, replace with actual computation)
 //   MatrixXf J(6, 20);
    // Replace this with your Jacobian calculation

    // Compute force F = J^-T * tau
 //   VectorXf F = J.transpose().inverse() * tauEst;

    // Print the calculated force
//    std::cout << "Calculated Force (F):\n" << F << std::endl;
//}

// Assuming jacobian is a 6x12 matrix, fill it with the calculated Jacobian

// Calculate pseudo-inverse of the Jacobian

//Eigen::MatrixXd jacobian_pinv = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse();

// Transpose the right pseudo-inverse

//Eigen::MatrixXd jacobian_pinv_transpose = jacobian_pinv.transpose();

// Calculate F = J^{R}T * tau

//Eigen::VectorXd force = jacobian_pinv_transpose * tau; 

// Print calculated force
//std::cout << "Calculated Force: " << force.transpose() << std::endl;



//void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg) {
    // Extract tauEst values from the first 12 motor states
 //   for (int i = 0; i < 12; ++i) {
 //       float tauEst = msg->motorState[i].tauEst;
        // Print tauEst for the ith motor
 //       ROS_INFO("tauEst for motor %d: %.4f", i, tauEst);
//    }
//}

// Define global variable to store tauEst values
// Eigen::VectorXd tau(12);
// KDL::Jacobian jac;
// std::vector<double> joint_positions(12, 0.0);

// void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg) {
//     // Extract tauEst values from the first 12 motor states and store them in tau vector
//     for (int i = 0; i < 12; ++i) {
//         joint_positions[i] = msg->motorState[i].q;
//         tau(i) = msg->motorState[i].tauEst;
//     }
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_walk_without_lcm");

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh;

    ros::Rate loop_rate(500);

    long motiontime = 0;

    unitree_legged_msgs::HighCmd high_cmd_ros;

    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);

//     KDL::Tree robot_kin;
//     kdl_parser::treeFromFile("go1.urdf", robot_kin);

//     KDL::Chain chain;
//     if (!robot_kin.getChain("base_link", "tip_link", chain)) {
//         ROS_ERROR("Failed to get chain from KDL tree");
//     return -1;
//     }

//     //Create the solver
//     KDL::ChainJntToJacSolver jac_solver(chain);

//     // Define the joint positions (assuming you have them)
//     KDL::JntArray joint_positions(chain.getNrOfJoints());

//     // Assuming you have the joint positions filled in the joint_positions array

//     // Create a Jacobian matrix
//     KDL::Jacobian jac(chain.getNrOfJoints());

//     // Solve the Jacobian for the given joint positions
//     jac_solver.JntToJac(joint_positions, jac);

//     // Print the Jacobian matrix
//     std::cout << "Jacobian Matrix:" << std::endl;
//     for (int i = 0; i < jac.columns(); ++i) {
//         std::cout << "Column " << i << ":" << std::endl;
//         for (int j = 0; j < jac.rows(); ++j) {
//             std::cout << jac(j, i) << "\t";
//         }
//         std::cout << std::endl;
// }


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
        

        // --------------TASK 2-------------
        // tau = ... (get it from the robot)
        // J = ...
        // F = J^-T * tau
        // print(F)
        
        pub.publish(high_cmd_ros);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}