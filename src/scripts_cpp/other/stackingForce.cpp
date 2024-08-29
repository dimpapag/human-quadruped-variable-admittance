#include <ros/ros.h>
#include <unitree_legged_msgs/HighState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include <iostream>

// Define vectors to store joint configurations and estimated torques for each leg
Eigen::VectorXd all_q(12), all_tauEst(12);
Eigen::Vector3d q_FR, q_FL, q_RR, q_RL;
Eigen::Vector3d tau_FR, tau_FL, tau_RR, tau_RL;
Eigen::Vector3d f_FR, f_FL, f_RR, f_RL; // Forces for each leg
Eigen::VectorXd F_A(12); // Stacked forces
Eigen::Vector3d p_FR, p_FL, p_RR, p_RL; // Joint positions for each leg

// Define callback function to handle high state messages
void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg)
{
    // Extract all joint configurations (q) and estimated joint torques (tauEst)
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

    // Compute forward kinematics for each leg
    KDL::Chain chain_FR, chain_FL, chain_RR, chain_RL;
    if (!tree.getChain("base", "FR_foot", chain_FR) ||
        !tree.getChain("base", "FL_foot", chain_FL) ||
        !tree.getChain("base", "RR_foot", chain_RR) ||
        !tree.getChain("base", "RL_foot", chain_RL)) {
        std::cerr << "Failed to extract chains from URDF." << std::endl;
        return 1;
    }

    KDL::ChainFkSolverPos_recursive fk_solver_FR(chain_FR);
    KDL::ChainFkSolverPos_recursive fk_solver_FL(chain_FL);
    KDL::ChainFkSolverPos_recursive fk_solver_RR(chain_RR);
    KDL::ChainFkSolverPos_recursive fk_solver_RL(chain_RL);

    // Main loop
    while (ros::ok()) {
        ros::spinOnce(); // Process any incoming messages

        // Compute forward kinematics for each leg
        KDL::Frame p_out_FR, p_out_FL, p_out_RR, p_out_RL;
        KDL::JntArray q_FR_kdl(3), q_FL_kdl(3), q_RR_kdl(3), q_RL_kdl(3);
        for (int i = 0; i < 3; ++i) {
            q_FR_kdl(i) = q_FR(i);
            q_FL_kdl(i) = q_FL(i);
            q_RR_kdl(i) = q_RR(i);
            q_RL_kdl(i) = q_RL(i);
        }

        fk_solver_FR.JntToCart(q_FR_kdl, p_out_FR);
        fk_solver_FL.JntToCart(q_FL_kdl, p_out_FL);
        fk_solver_RR.JntToCart(q_RR_kdl, p_out_RR);
        fk_solver_RL.JntToCart(q_RL_kdl, p_out_RL);

        // Extract position for each leg
        p_FR = Eigen::Vector3d(p_out_FR.p.x(), p_out_FR.p.y(), p_out_FR.p.z());
        p_FL = Eigen::Vector3d(p_out_FL.p.x(), p_out_FL.p.y(), p_out_FL.p.z());
        p_RR = Eigen::Vector3d(p_out_RR.p.x(), p_out_RR.p.y(), p_out_RR.p.z());
        p_RL = Eigen::Vector3d(p_out_RL.p.x(), p_out_RL.p.y(), p_out_RL.p.z());

        // Print positions for each joint
        std::cout << "Position of joints for FR leg: " << p_FR.transpose() << std::endl;
        std::cout << "Position of joints for FL leg: " << p_FL.transpose() << std::endl;
        std::cout << "Position of joints for RR leg: " << p_RR.transpose() << std::endl;
        std::cout << "Position of joints for RL leg: " << p_RL.transpose() << std::endl;

        // Compute Jacobian for each leg using dynamic q values
        KDL::ChainJntToJacSolver jac_solver_FR(chain_FR);
        KDL::ChainJntToJacSolver jac_solver_FL(chain_FL);
        KDL::ChainJntToJacSolver jac_solver_RR(chain_RR);
        KDL::ChainJntToJacSolver jac_solver_RL(chain_RL);

        KDL::Jacobian jac_FR(chain_FR.getNrOfJoints());
        KDL::Jacobian jac_FL(chain_FL.getNrOfJoints());
        KDL::Jacobian jac_RR(chain_RR.getNrOfJoints());
        KDL::Jacobian jac_RL(chain_RL.getNrOfJoints());

        KDL::JntArray q_array_FR(3), q_array_FL(3), q_array_RR(3), q_array_RL(3);
        for (int i = 0; i < 3; ++i) {
            q_array_FR(i) = q_FR(i);
            q_array_FL(i) = q_FL(i);
            q_array_RR(i) = q_RR(i);
            q_array_RL(i) = q_RL(i);
        }

        jac_solver_FR.JntToJac(q_array_FR, jac_FR);
        jac_solver_FL.JntToJac(q_array_FL, jac_FL);
        jac_solver_RR.JntToJac(q_array_RR, jac_RR);
        jac_solver_RL.JntToJac(q_array_RL, jac_RL);

        // Compute inverse and transpose for each leg
        Eigen::MatrixXd J_inv_FR = jac_FR.data.topLeftCorner(3, jac_FR.columns()).transpose().inverse();
        Eigen::MatrixXd J_inv_FL = jac_FL.data.topLeftCorner(3, jac_FL.columns()).transpose().inverse();
        Eigen::MatrixXd J_inv_RR = jac_RR.data.topLeftCorner(3, jac_RR.columns()).transpose().inverse();
        Eigen::MatrixXd J_inv_RL = jac_RL.data.topLeftCorner(3, jac_RL.columns()).transpose().inverse();

        // Compute forces for each leg
        f_FR = J_inv_FR * tau_FR;
        f_FL = J_inv_FL * tau_FL;
        f_RR = J_inv_RR * tau_RR;
        f_RL = J_inv_RL * tau_RL;

        // Print forces for each leg
        std::cout << "Force for FR leg: " << f_FR.transpose() << std::endl;
        std::cout << "Force for FL leg: " << f_FL.transpose() << std::endl;
        std::cout << "Force for RR leg: " << f_RR.transpose() << std::endl;
        std::cout << "Force for RL leg: " << f_RL.transpose() << std::endl;

        // Stack forces
        F_A << f_FR, f_FL, f_RR, f_RL;

        // Other computations...
        
        loop_rate.sleep(); // Control the loop rate
    }

    return 0;
}
