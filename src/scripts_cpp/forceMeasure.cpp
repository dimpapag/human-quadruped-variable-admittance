#include <ros/ros.h>
#include <unitree_legged_msgs/HighState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sys/types.h>
#include <ctime>
#include <chrono>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>

# define M_PI           3.14159265358979323846  /* pi */

// Define vectors to store joint configurations and estimated torques for each leg
Eigen::VectorXd all_q(12), all_tauEst(12);
Eigen::Vector3d q_FR, q_FL, q_RR, q_RL;
Eigen::Vector3d tau_FR, tau_FL, tau_RR, tau_RL;
Eigen::Vector3d f_FR, f_FL, f_RR, f_RL; // Forces for each leg
Eigen::VectorXd F_A(12); // Stacked forces
Eigen::VectorXd F_C(6); // Resultant force vector
Eigen::VectorXd f_f(6); // Resultant force vector
Eigen::Vector3d p_FR, p_FL, p_RR, p_RL; // Joint positions for each leg
Eigen::MatrixXd G(6, 12); //Gamma Matrix

// Quaternion data
float e_x, e_y, e_z, etta;

double wc =2*M_PI*0.1;

// Define gravity vector
Eigen::Vector3d Wg(0, 0, -117.02);

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

    e_x=msg->imu.quaternion[0];
    e_y = msg->imu.quaternion[1];
    e_z = msg->imu.quaternion[2];
    etta = msg->imu.quaternion[3];

}

// Compute skew-symmetric matrix from a given vector
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d skew_sym;
    skew_sym <<    0, -v(2),  v(1),
                 v(2),     0, -v(0),
                -v(1),  v(0),    0;
    return skew_sym;
}

// Function to get the current timestamp as a string
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&timestamp), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

// Print joint names for a given KDL Chain
void printChain(const KDL::Chain& chain) {
    std::cout << "Joint names for the chain:" << std::endl;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        std::string joint_name = chain.getSegment(i).getJoint().getName();
        std::cout << joint_name << std::endl;
    }
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "kinematics_dynamics_calculator");
    ros::NodeHandle nh;

    // Subscribe to high state topic
    ros::Subscriber high_state_sub = nh.subscribe<unitree_legged_msgs::HighState>(
        "/high_state", 1, highStateCallback
    );

    // Define loop rate (adjust as needed)
    ros::Rate loop_rate(500); // 500 Hz

    // Ensure the directory exists, create if it doesn't
    std::string directory = "csv_Rec_Folder";
    struct stat info;

    if (stat(directory.c_str(), &info) != 0) {
        if (mkdir(directory.c_str(), 0777) == -1) {
            std::cerr << "Error creating directory: " << strerror(errno) << std::endl;
            return 1;
        }
    } else if (!(info.st_mode & S_IFDIR)) {
        std::cerr << directory << " is not a directory" << std::endl;
        return 1;
    }

    // Create a unique filename with timestamp
    std::string filename = directory + "/forceCSV_" + getCurrentTimestamp() + ".csv";

    // Open file to log F_C values
    std::ofstream fc_log(filename);
    if (!fc_log.is_open()) {
        std::cerr << "Failed to open file for logging." << std::endl;
        return 1;
    }

    // Write CSV header
    fc_log << "time(sec),F_C0,F_C1,F_C2,F_C3,F_C4,F_C5\n";

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

    //     // Print joint names for each leg
    // std::cout << "Joint names for FR leg:" << std::endl;
    // printChain(chain_FR);
    // std::cout << "Joint names for FL leg:" << std::endl;
    // printChain(chain_FL);
    // std::cout << "Joint names for RR leg:" << std::endl;
    // printChain(chain_RR);
    // std::cout << "Joint names for RL leg:" << std::endl;
    // printChain(chain_RL);


    KDL::ChainFkSolverPos_recursive fk_solver_FR(chain_FR);
    KDL::ChainFkSolverPos_recursive fk_solver_FL(chain_FL);
    KDL::ChainFkSolverPos_recursive fk_solver_RR(chain_RR);
    KDL::ChainFkSolverPos_recursive fk_solver_RL(chain_RL);

    // Main loop
    double dt = 1.0 / 500.0; // Time step based on loop rate (500 Hz)
    double t = 0.0; // Initial time
    f_f= Eigen::VectorXd::Zero(6);
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
        // std::cout << "Position of joints for FR leg: " << p_FR.transpose() << std::endl;
        // std::cout << "Position of joints for FL leg: " << p_FL.transpose() << std::endl;
        // std::cout << "Position of joints for RR leg: " << p_RR.transpose() << std::endl;
        // std::cout << "Position of joints for RL leg: " << p_RL.transpose() << std::endl;

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
        // std::cout << "Force for FR leg: " << f_FR.transpose() << std::endl;
        // std::cout << "Force for FL leg: " << f_FL.transpose() << std::endl;
        // std::cout << "Force for RR leg: " << f_RR.transpose() << std::endl;
        // std::cout << "Force for RL leg: " << f_RL.transpose() << std::endl;

        // Stack forces
        F_A << f_FR, f_FL, f_RR, f_RL;

        // Compute skew-symmetric matrices for position vectors
        Eigen::Matrix3d S_FR = skewSymmetric(p_FR);
        Eigen::Matrix3d S_FL = skewSymmetric(p_FL);
        Eigen::Matrix3d S_RR = skewSymmetric(p_RR);
        Eigen::Matrix3d S_RL = skewSymmetric(p_RL);

        // Print skew-symmetric matrices for each leg
        // std::cout << "Skew-symmetric matrix for FR leg:\n" << S_FR << std::endl;
        // std::cout << "Skew-symmetric matrix for FL leg:\n" << S_FL << std::endl;
        // std::cout << "Skew-symmetric matrix for RR leg:\n" << S_RR << std::endl;
        // std::cout << "Skew-symmetric matrix for RL leg:\n" << S_RL << std::endl;

        // Create G matrix
        G << Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Identity(3, 3),
             S_FR, S_FL, S_RR, S_RL;

        // Print G
        //std::cout << "Gamma Matrix :\n" << G << std::endl;

        // Calculate resultant force vector F_C
        F_C = G * F_A;

        // Convert quaternion to KDL rotation matrix
        KDL::Rotation R_C0 = KDL::Rotation::Quaternion(e_x, e_y, e_z, etta);

        // Optionally, convert KDL::Rotation to Eigen::Matrix3d if needed
        Eigen::Matrix3d R_C0_eigen;
        R_C0_eigen << R_C0(0,0), R_C0(0,1), R_C0(0,2),
                      R_C0(1,0), R_C0(1,1), R_C0(1,2),
                      R_C0(2,0), R_C0(2,1), R_C0(2,2);

        // // Print rotation matrix
         //std::cout << "Rotation Matrix R_C0:\n" << R_C0_eigen << std::endl;

        //Gravity Compensation
        F_C.segment<3>(0) -= R_C0_eigen.transpose() * Wg;

        // // Print F_C
        std::cout << "Resultant force vector F_C:" << std::setprecision(2) << F_C(1) << std::endl;
        
        if(F_C(0)==F_C(0)){
            f_f = (1.0-wc*dt) *f_f + wc * dt * F_C;
        }

        //std::cout << "Resultant force vector f_f:\n" << std::setprecision(2) << f_f.transpose() << std::endl;

        // Log F_C values to CSV with timestamp incremented by dt
        fc_log << std::fixed << std::setprecision(6) << t << ","
               << f_f(0) << "," << f_f(1) << "," << f_f(2) << ","
               << f_f(3) << "," << f_f(4) << "," << f_f(5) << "\n";
    
        
        // Increment time
        t += dt;

        loop_rate.sleep(); // Control the loop rate
    }

    // Close the CSV file
    fc_log.close();

    return 0;
}