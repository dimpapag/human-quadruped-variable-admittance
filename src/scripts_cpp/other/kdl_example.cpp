#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <iostream>

int main() {
    // Define a simple kinematic chain with two joints
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.0, 0.0, 0.1))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.0, 0.0, 0.1))));

    // Create a forward kinematics solver
    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    // Create a Jacobian solver
    KDL::ChainJntToJacSolver jac_solver(chain);

    // Define joint positions
    KDL::JntArray q(chain.getNrOfJoints());
    q(0) = 0.1;  // Joint 1 position (radians)
    q(1) = 0.2;  // Joint 2 position (radians)

    // Compute forward kinematics
    KDL::Frame end_effector_pose;
    fk_solver.JntToCart(q, end_effector_pose);

    // Compute Jacobian
    KDL::Jacobian jac;
    jac_solver.JntToJac(q, jac);

    // Output results
    std::cout << "End-effector position: " << end_effector_pose.p << std::endl;
    std::cout << "End-effector orientation: " << end_effector_pose.M << std::endl;
    std::cout << "Jacobian matrix:\n" << jac.data << std::endl;

    return 0;
}