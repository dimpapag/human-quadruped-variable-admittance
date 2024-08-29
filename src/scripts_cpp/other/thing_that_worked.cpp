
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
using namespace Eigen;
using namespace KDL;

// Define Eigen typedefs for convenience
typedef Matrix<float, 3, 1> Vector3f;
typedef Matrix<float, 6, 3> Matrix6x3f;

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "jacobian_computation");
    ros::NodeHandle nh;

    // Load URDF model
    urdf::Model model;
    if (!model.initFile("/home/con/catkin_ws/src/unitree_ros/robots/go1_description/urdf/go1.urdf")) {
        ROS_ERROR("Failed to parse URDF file");
        return -1;
    }

    // Create KDL chain for the Front Right leg
    Chain chain_FR;
    Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
        ROS_ERROR("Failed to construct KDL tree from URDF model");
        return -1;
    }

    if (!tree.getChain("base", "FR_foot", chain_FR)) {
        ROS_ERROR("Failed to get chain from KDL tree");
        return -1;
    }

    // Joint positions for the Front Right leg
    JntArray q_FR(chain_FR.getNrOfJoints());
    q_FR(0) = 0.0; // Set the joint position value for the first joint, adjust this based on your robot
    q_FR(1) = 0.0;
    q_FR(2) = 0.0;
    // Define the Jacobian solver
    ChainJntToJacSolver jac_solver(chain_FR);

    // Compute the Jacobian
    Jacobian J_FR(chain_FR.getNrOfJoints());
    jac_solver.JntToJac(q_FR, J_FR);

    // Print the Jacobian
    std::cout << "Jacobian for Front Right leg (J_FR):\n" << J_FR.data << std::endl;

    return 0;
}
