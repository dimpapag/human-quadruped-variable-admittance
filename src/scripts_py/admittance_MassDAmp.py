#!/usr/bin/env python3

# from __future__ import print_function

import roslib #; roslib.load_manifest('champ_teleop')
import rospy
import time

from CSRL_orientation import *

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose as Pose
from nav_msgs.msg import Odometry as Odometry
import tf

import sys, select, termios, tty
import numpy as np
import matplotlib.pyplot as plt

import os
import csv

class Admit_controller:
    def __init__(self):


        self.R0b = np.identity(3)

        # Initialize ROS publishers for velocity and pose
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self.dt = 0.002
        self.control_rate = rospy.Rate(1.0/self.dt)

        # Initialize empty lists to store data
         #time
        self.log_t = 0 
         #velocity
        self.log_vb = 0
        #self.log_vb = np.zeros(3)
        #self.log_Pd_dot = np.zeros(3)
        #self.log_e = np.zeros(3)
         # Force
        self.log_Fext = np.zeros(3)
        
        # Folder path for saving figures
        self.figures_folder = 'Trajfigures'
        if not os.path.exists(self.figures_folder):
            os.makedirs(self.figures_folder)
            
        # Folder path for saving variable data
        self.folder_path = 'TrajcsvFolder'
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)

    def save_data_to_csv(self):
        timestamp = time.strftime("%Y%m%d-%H%M%S")  # Generates a unique timestamp
        
        # Creating file paths
        velocity_file_path = os.path.join(self.folder_path, f"vel_{timestamp}.csv")
        
        # Save velocity data 
        with open(velocity_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time", "Velocity_norm"])
            for i in range(len(self.log_t)):
                writer.writerow([self.log_t[i, 0], self.log_vb[i, 0]])

        print(f"Data saved in folder: {self.folder_path}")

    def save_figure(self, figure, name_prefix):
        timestamp = time.strftime("%Y%m%d-%H%M%S")  # Generates a unique timestamp
        file_path = os.path.join(self.figures_folder, f'{name_prefix}_{timestamp}.png')
        figure.savefig(file_path)
        print(f"Figure saved: {file_path}")

    
    def external_force(t, tStart, duration=5):
        if tStart < t and t < tStart + duration:
            fx = 10.0  # Specify your desired force in x direction
            fy = 0.0  # Specify your desired force in y direction
            tauZ = 0.0  # Specify your desired torque around z axis
            return np.array([fx, fy, tauZ])
        else:
            return np.array([0, 0, 0])

    def control_loop(self):
        
        # Intialize twist
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        v = np.zeros(3) # initializing velocity for python reasons 

        t = 0.0
        total=10.0
        #pulse duration 
        dur=5 # 5sec 
        tStart =3 
        tStop=tStart+dur

        #Admittance desired characteristics: 
         #Desired Inertia
        M_d=np.array([[10,0,0],
              [0,10,0],
              [0,0, 2]])
         #Desired Damping
        D_d=np.array([[40,0,0],
              [0,40,0],
              [0,0,20]])

        
        #Initializing matrices A,B of linear Differential equation:
        A = - np.linalg.inv(M_d) @  D_d
        B = np.linalg.inv(M_d)

        # Zeroing the Admittance state for python reasons
        v_ad = np.zeros(3)   
        v_ad_dot = np.zeros(3)


        # actual control loop
        while not rospy.is_shutdown() and t<=total:

            # Consider F (external force):
            # Fext=[Fx,Fy ,tau_z] Generalized Force
            Fext = np.zeros(3) # "else" case
            if t < 3:  # F=F(t)
                Fext[0] = 30
                Fext[1] = 30
                #Fext[2] = 0
            
            # Calculating deviation from virtual target
            # x = np.array([self.p-pVT,self.theta-thetaVT])
        
            # Calculate desired acceleration using admittance controller equation
            #M_d*x_ddot+D_d*x_dot+Kp*x=Fext
            #x_ddot = np.linalg.solve(M_d, Fext - np.dot(D_d, x_dot) - np.dot(K_d, x))
            v_ad = v_ad + v_ad_dot * self.dt
            v_ad_dot = A @ v_ad + B @ Fext
            
    
            #Same for their first order derivatives
            p_dot_ref = v_ad[0:2] 
            theta_dot_ref = v_ad[2] 

            t = t + self.dt
            self.log_t = np.vstack((self.log_t,t))  # logging time
            # print(time.time() - t_now)
            # t_now =  time.time()

            print(t)
           

            v[0:2] = p_dot_ref
            v[2] = 0.0
            
            vb =  v #V_body = Gamma_body,base * V_base
            
            wb =  theta_dot_ref # Omega_body

            # Log variables
             # Velocities
            self.log_vb = np.vstack((self.log_vb,np.linalg.norm(vb)))  # Log velocity
            #self.log_vb = np.vstack((self.log_vb,vb))  # Log velocity
            #self.log_Pd_dot = np.vstack((self.log_Pd_dot,vb))    # Log target velocity
             # Errors
            #self.log_e = np.vstack((self.log_e,e)) 
             # External force
            self.log_Fext = np.vstack((self.log_Fext,Fext))
            # send commands
            twist.linear.x = vb[0]
            twist.linear.y = vb[1]
            twist.angular.z = wb

            self.velocity_publisher.publish(twist)

            # synchronize loop
            self.control_rate.sleep()
       
if __name__ == "__main__":
    # Initialize ROS node and Teleop object
    rospy.init_node('admit_controller')
    ac = Admit_controller()

    ac.control_loop()

    # Saving the logged variables to CSV files
    #ac.save_data_to_csv()

    # Plotting the logged variables
    #Plotting Velocity
    ac.fig2 = plt.figure()
    plt.plot(ac.log_t, ac.log_vb, label='||vb||')
    #plt.plot(ac.log_t, ac.log_v[:, 0], label='Velocity x')
    #plt.plot(ac.log_t, ac.log_pT_dot[:, 0], label='Target Velocity x',color='green',linestyle='--')
    plt.title("Norm of Linear Velocity")
    plt.xlabel('Time(s)')
    plt.ylabel('||Velocity||(m/s)')
    plt.legend()
    plt.grid(True)
    ac.save_figure(ac.fig2, 'trajectory_tracking_v')

    plt.show()