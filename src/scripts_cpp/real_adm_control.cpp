#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
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
#include <std_msgs/Float32MultiArray.h> //keyboard message
#include "command_give/F_external.h" //sensor message
//#include <csignal>
#include <cstdlib>
#include <signal.h>

using namespace UNITREE_LEGGED_SDK;
using namespace Eigen;

ros::Publisher velocity_publisher;
double dt = 0.002; // Time step based on loop rate (500 Hz)
double t = 0.0; // Initial time
Vector2d vb = Vector2d::Zero(); // Initial velocity (x, y)
double wb = 0.0; // Initial Yaw speed
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

// For the calculation of Fv : 
double k_m = 1.0;
double k_a = 10.0;
double k_q = 1.0;
bool enable_fv = false;
double f_0 = 20.0;
double f_m =0.0;
double betta = 0.04;
double kappa = 20.0;
double S_TOT = 0.0;
double h =0.0;
Vector3d v_linear_recieved = Vector3d::Zero();
Vector4d c=Vector4d::Zero();
Vector4d mu=Vector4d::Zero();
//To ensure the values f_i are accessible for calculation within the main loop:
float f_1 = 0.0, f_2 = 0.0, f_3 = 0.0, f_4 = 0.0, f_min = 48.0, f_vac=0.0;
double f_v = 0.0;
Vector3d F_v = Vector3d::Zero();
double a=0.0;
void chatterCallback(const command_give::F_external::ConstPtr& msg)
{
//   ROS_INFO("Received F_external:");
//   for (size_t i = 0; i < msg->data.size(); ++i)
//   {
//     ROS_INFO(" %f", msg->data[i]);
//   }
  // Rename and assign the received force values

    float f_x = msg->data[0];
    float f_y = msg->data[1];
    float tau_z = msg->data[5];
    // Assign to Fext
    Fext(0) = -f_x;
    Fext(1) = -f_y;
    Fext(2) = -tau_z;
    
    // float tau_y = msg->data[4];
    f_1 = msg->data[6];
    f_2 = msg->data[7];
    f_3 = msg->data[8];
    f_4 = msg->data[9];
    f_vac = msg->data[10];
}

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg) {
    // // Handle high state update if needed
    // for (int i=0; i<3 ; i++) {
    //     v_linear_recieved(i) = msg->velocity[i];
    // }
    // // Print V_linear 
    // std::cout << "Recieved velocity from HighState V_lin:\n" << std::setprecision(4) << v_linear_recieved.transpose() << std::endl;

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


std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&timestamp), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

// Define a global ofstream object for logging
std::ofstream info_log;
unitree_legged_msgs::HighCmd high_cmd_ros;
ros::Publisher pub;
void signalHandler( int signum ) { // Oti uparxei edw mesa htan katw apo thn while
        std::cout << "Interrupt signal (" << signum << ") received." << std::endl;

        high_cmd_ros.velocity[0] = 0.0; // X-axis velocity
        high_cmd_ros.velocity[1] = 0.0; // Y-axis velocity
        high_cmd_ros.yawSpeed = 0.0; // Yaw speed 
        pub.publish(high_cmd_ros);
            if (info_log.is_open())
        info_log.close();

        ros::shutdown();
   // terminate program  

    exit(signum);  
    }

int main(int argc, char** argv) {

    signal(SIGINT, signalHandler); // Handle Ctrl+C
    
    struct sigaction sigIntHandler;

    // sigIntHandler.sa_handler = signalHandler;
    // sigemptyset(&sigIntHandler.sa_mask);
    // sigIntHandler.sa_flags = 0;

    // sigaction(SIGINT, &sigIntHandler, NULL);

    //ros::init(argc, argv, "admit_controller");
    ros::init(argc, argv, "admit_controller",ros::init_options::NoSigintHandler);
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

    //ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);
    pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);
    ros::Subscriber high_state_sub = nh.subscribe<unitree_legged_msgs::HighState>(
        "/high_state", 1, highStateCallback
    );
    ros::Subscriber sub = nh.subscribe("F_external", 1000, chatterCallback);

    //unitree_legged_msgs::HighCmd high_cmd_ros;

    if (enable_logging) {
        // Create directory if it does not exist
        std::string directory = "/home/con/catkin_ws/src/command_give/Real_Admittance_Logging";
        struct stat info;
        if (stat(directory.c_str(), &info) != 0) {
            // Directory does not exist, create it
            if (mkdir(directory.c_str(), 0777) == -1) {
                std::cerr << "Error: Failed to create directory " << directory << std::endl;
                return 1;
            }
        }

        // Create log file
        std::string filename = directory + "/real_adm_csv_" + getCurrentTimestamp() + ".csv";
        info_log.open(filename);
        if (!info_log.is_open()) {
            std::cerr << "Error: Failed to create log file " << filename << std::endl;
            return 1;
        }

        // Write header to log file
        info_log << "time(sec),v_x(m/s),v_y(m/s),omega_z(rad/s),f_x(N),f_y(N),tau_z(Nm),p_x(m),p_y(m),theta_z(rad),Power(Watt),d_factor,f_1(N),f_2(N),f_3(N),f_4(N),f_m(N),-f_v" << std::endl;
    }

    ros::Rate loop_rate(500); // 500 Hz
    // Publish initial zero velocities
    initializing_robot(high_cmd_ros);
    pub.publish(high_cmd_ros);


        // Desired Admittance Characteristics
    // Target Inertia 
    M_d(0, 0) = 13.0;
    M_d(1, 1) = 13.0;
    M_d(2, 2) = 1.5;

    // Initial Damping 
    double d_fac = 1.0;
    double d_low = 0.4;
    double d_high = 2.0;
    double lambda = 3.0;

    D_d(0, 0) = 20.0;
    D_d(1, 1) = 20.0;
    D_d(2, 2) = 5.0;

    Eigen::Matrix3d D_var = D_d;
    double P_plus=0.0;

    M_d_inv = M_d.inverse();
    
    B = M_d_inv;

    // Saturation limits for vb and wb
    //const double vb_max = 0.5;
    const double vb_max = 1.0;
    const double vb_min = -vb_max;
    //const double wb_max = 3.52; 
    const double wb_max = 3.52;
    const double wb_min = -wb_max; 

    //For the partial stuff
    const double bk=betta * kappa;
    c << -bk,bk,bk,-bk;

    while (ros::ok()) {

        // Calculate h(f_1, f_2, f_3, f_4)
        S_TOT = exp(-f_1 / k_m) + exp(-f_2 / k_m) + exp(-f_3 / k_m) + exp(-f_4 / k_m);
        h = -k_m * log(S_TOT);
        //std::cout << "f_1:" << std::endl <<  f_1 << std::endl;
        // Calculate f_m
        f_m = f_min + h;
        std::cout << "f_m:" << std::endl <<  f_m  << std::endl;


  

        // Calculate (theta(W)/theta(fm)) --> a ( scalar)
        if(f_m < f_0){
            a = k_a * ((1.0 / (f_0 * pow(f_m, 2))) - (1.0 / pow(f_m, 3))) + k_q * (f_m - f_0 );
         
        }else{
            a = 0.0;
        }
        // Calculate (theta(fm)/theta(*f*)) --> b (row vector)
        mu(0) = exp(-f_1 / k_m) / S_TOT;
        mu(1) = exp(-f_2 / k_m) / S_TOT;
        mu(2) = exp(-f_3 / k_m) / S_TOT;
        mu(3) = exp(-f_4 / k_m) / S_TOT;
        // theta(f)/theta(theta) --> c (column vector) has been calcluated (it's a constant vector)
        f_v = ((double)enable_fv) * a * mu.dot(c) ;

        std::cout << "-f_v:" << std::endl <<  -f_v  << std::endl;
        F_v << 0, 0, -f_v;
        A = -M_d_inv * D_var;

        // Admittance control calculation
        v_ad = v_ad + v_ad_dot * dt;
        v_ad_dot = A * v_ad + B * ( Fext + F_v );
        theta = theta + wb * dt;
        Rz(0,0) =  std::cos(theta);
        Rz(0,1) = -std::sin(theta);
        Rz(1,0) = std::sin(theta);
        Rz(1,1) = std::cos(theta);
        pose.head<2>() = pose.head<2>() + Rz * v_ad.head<2>() * dt; //world_frame_p=base_p+Rot(z,theta)*v_ad*dt
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

        P_plus = std::max(0.0, power);
        d_fac = d_low + (d_high - d_low) * exp(-lambda*P_plus);  
        //d_fac = d_low;
        //d_fac = d_high;
        D_var = d_fac * D_d;

        //std::cout << "D_var:" << std::endl <<  D_var  << std::endl;

        //std::cout << "Time: " << t << ", Velocity: (" << vb(0) << ", " << vb(1) << "), Yaw Speed: " << wb << ", Power: " << power << std::endl;
        // std::cout << "Forces" << Fext.transpose() << std::endl;
        // Publish velocity commands
        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.levelFlag = HIGHLEVEL;
        high_cmd_ros.mode = 2; 
        high_cmd_ros.gaitType = 1;

        high_cmd_ros.velocity[0] = 1.0*vb(0); // X-axis velocity
        //std::cout << "linnnn x:" << high_cmd_ros.velocity[0] << std::endl;
        high_cmd_ros.velocity[1] = 1.0*vb(1); // Y-axis velocity
        //std::cout << "linnnn y:" << high_cmd_ros.velocity[0] << std::endl;
        high_cmd_ros.yawSpeed = 1.0*wb; // Yaw speed
        // std::cout << "HIGH_CMD_YAW:" << high_cmd_ros.yawSpeed  << std::endl;
        // std::cout << "WB:" << wb  << std::endl;
        //high_cmd_ros.velocity[0] = 0.0; // X-axis velocity
        //high_cmd_ros.velocity[1] = 0.0; // Y-axis velocity
        //high_cmd_ros.yawSpeed = 0.0; // Yaw speed

        pub.publish(high_cmd_ros);
        // Log data if logging is enabled
        if (enable_logging) {
            info_log << std::fixed << std::setprecision(6) << t << "," << vb(0) << "," << vb(1) << "," << wb << "," << Fext(0) << "," << Fext(1) << "," << Fext(2) << "," << pose(0) << "," << pose(1) << "," << pose(2) << "," << power << "," << d_fac << "," << f_1 << "," << f_2 << "," << f_3 << "," << f_4 << "," << f_m << "," << -f_v <<std::endl;
        }
        t += dt; // Increment time
        loop_rate.sleep(); // Control the loop rate
        ros::spinOnce();
    }

    return 0;
}


