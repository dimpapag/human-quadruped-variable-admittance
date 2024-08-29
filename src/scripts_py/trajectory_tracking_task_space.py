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

        #Initializations
        self.p = np.zeros(3)
        self.theta = 0.0

        self.R0b = np.identity(3)

        # Initialize ROS publishers for velocity and pose
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self.dt = 0.002
        self.control_rate = rospy.Rate(1.0/self.dt)

        # Initialize empty lists to store data
        self.log_t = 0 
        self.log_vb = 0
        #self.log_wb = 0
        #self.log_vb = np.zeros(3)
        self.log_p = np.zeros(3)
        self.log_pT = np.zeros(3)
        self.log_theta=0
        self.log_thetaT=0
        #self.log_pT_dot = np.zeros(3)
        self.log_e = 0
        #self.log_e = np.zeros(3)
        self.log_e_o=0
        

        # Folder path for saving figures
        self.figures_folder = 'Adm_figures'
        if not os.path.exists(self.figures_folder):
            os.makedirs(self.figures_folder)
            
        # Folder path for saving variable data
        self.folder_path = 'Adm_csvData'
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)

    def save_data_to_csv(self):
        timestamp = time.strftime("%Y%m%d-%H%M%S")  # Generates a unique timestamp
        
        # Create file paths
        pos_file_path = os.path.join(self.folder_path, f"pos_{timestamp}.csv")
        velocity_file_path = os.path.join(self.folder_path, f"vel_{timestamp}.csv")
        error_file_path = os.path.join(self.folder_path, f"error_{timestamp}.csv")
        
        # Save position data
        with open(pos_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time", "Position_x", "Position_y", "Position_z", "Target_Position_x", "Target_Position_y", "Orientation_z", "Target_Orientation_z"])
            for i in range(len(self.log_t)):
                writer.writerow([self.log_t[i, 0], self.log_p[i, 0], self.log_p[i, 1], self.log_p[i, 2], self.log_pT[i, 0], self.log_pT[i, 1], self.log_theta[i, 0], self.log_thetaT[i, 0]])
        
        # Save velocity data
        with open(velocity_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time", "Velocity_norm"])
            for i in range(len(self.log_t)):
                writer.writerow([self.log_t[i, 0], self.log_vb[i, 0]])
        
        # Save error data
        with open(error_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time", "Error_norm"])
            for i in range(len(self.log_t)):
                writer.writerow([self.log_t[i, 0], self.log_e[i, 0]])

        print(f"Data saved in folder: {self.folder_path}")

    def save_figure(self, figure, name_prefix):
        timestamp = time.strftime("%Y%m%d-%H%M%S")  # Generates a unique timestamp
        file_path = os.path.join(self.figures_folder, f'{name_prefix}_{timestamp}.png')
        figure.savefig(file_path)
        print(f"Figure saved: {file_path}")

    def pose_callback(self, data):
        # read pose 
        self.p[0] =  data.pose.pose.position.x
        self.p[1] =  data.pose.pose.position.y
        self.p[2] =  data.pose.pose.position.z
        
        Q = np.zeros(4)
        Q[0] =  data.pose.pose.orientation.w
        Q[1] =  data.pose.pose.orientation.x
        Q[2] =  data.pose.pose.orientation.y
        Q[3] =  data.pose.pose.orientation.z

        self.R0b = quat2rot(Q)

        # projectin to the null space of z 
        Nz = np.identity(3)
        Nz[2,2] = 0 

        ktheta = angle_axis_vectors( np.array([1, 0, 0]),  Nz @ self.R0b[:, 0])
        self.theta = ktheta[2]

    def control_loop(self):
        
        # Intialize twist
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        #Controller Gains: 
        kp = 0.5
    
        # t_now = time.time()

        t = 0.0
        total=10.0
        
        #Trajectory to Track : Circle :
        r=0.8
        numofcircles=1
        f=numofcircles/total
        omega=2*np.pi*f #rad/s


        theta0 = self.theta

        # actual control loop
        while not rospy.is_shutdown() and t<=total:

            # Consider F 
            # Euler integration of impedance model for a given F -> compute x -> get pT, thetaT, pTdot, thetaTdot, 
            
            pT=np.array([r*np.cos(omega*t),r*np.sin(omega*t),self.p[2]])
            thetaT = theta0 +  np.pi/3 * np.sin(omega*t)

            pT_dot=np.array([-r*omega*np.sin(omega*t),r*omega*np.cos(omega*t),0])
            thetaT_dot= np.pi/3*omega*np.cos(omega*t)

            t = t + self.dt
            self.log_t = np.vstack((self.log_t,t))  # logging time
            # print(time.time() - t_now)
            # t_now =  time.time()

            print(t)
            # Control signal
            e = self.p - pT
            e_o= self.theta - thetaT
            e[2] = 0.0
            #self.log_e = np.vstack((self.log_e,e)) 
            self.log_e = np.vstack((self.log_e,np.linalg.norm(e))) 

            v = pT_dot - kp * e # pd_dot-kp(e)
            
            vb = self.R0b.T @ v #V_body = Gamma_body,base * V_base
            
            wb =  thetaT_dot - kp * e_o # Omega_body

            # Log variables
            #self.log_vb = np.vstack((self.log_vb,vb))  # Log velocity
            self.log_vb = np.vstack((self.log_vb,np.linalg.norm(vb)))  # Log velocity
            self.log_p = np.vstack((self.log_p,self.p))    # Log actual position
            self.log_pT = np.vstack((self.log_pT,pT))    # Log target position
            #self.log_pT_dot = np.vstack((self.log_pT_dot,vb))    # Log target velocity
            self.log_theta= np.vstack((self.log_theta,self.theta)) # Log actual orientation
            self.log_thetaT = np.vstack((self.log_thetaT,thetaT)) # Log target orientation
            

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
    ac.save_data_to_csv()

    # Plotting the logged variables

    #Plotting a specific position 
    #ac.fig1 = plt.figure()
    #plt.plot(ac.log_t, ac.log_p[:, 0], label='Position x')
    #plt.plot(ac.log_t, ac.log_pT[:, 0], label='Target Position x',color='green',linestyle='--')
    #plt.plot()
    #plt.title("Trajectory Tracking")
    #plt.xlabel('Time')
    #plt.ylabel('Position (x)')
    #plt.legend()
    #plt.grid(True)
    #ac.save_figure(ac.fig1, 'trajectory_tracking_p')

    #Plotting generalised position
    ac.fig4,(ax1,ax2,ax3) = plt.subplots(3,1)
    ac.fig4.suptitle("Generalised position")
    ax1.plot(ac.log_t,  ac.log_p[:, 0],  label='Pos_x')
    ax1.plot(ac.log_t, ac.log_pT[:, 0], label='Target Pos_x',color='red',linestyle='--')
    ax1.legend()
    ax1.grid(True)
    ax2.plot(ac.log_t,  ac.log_p[:, 1],  label='Pos_y')
    ax2.plot(ac.log_t, ac.log_pT[:, 1], label='Target Pos_y',color='green',linestyle='--')
    ax2.legend()
    ax2.grid(True)
    ax3.plot(ac.log_t, ac.log_theta,    label='Orient_z')
    ax3.plot(ac.log_t, ac.log_thetaT,   label='Target Orient_z',color='magenta',linestyle='--')
    ax3.legend()
    ax3.grid(True)
    #ax1.set_xlabel('t(sec)')
    #ax2.set_xlabel('t(sec)')
    ax3.set_xlabel('t(sec)')
    ax1.set_ylabel('x(m)')
    ax2.set_ylabel('y(m) 1')
    ax3.set_ylabel('theta_z (rad)')
    ac.save_figure(ac.fig4, 'trajectory_tracking_p')

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
    #Plotting position error
    ac.fig3 = plt.figure()
    #plt.plot(ac.log_t, ac.log_e[:, 0])
    plt.plot(ac.log_t, ac.log_e,label='||e||')
    plt.title("Norm of Error")
    plt.xlabel('Time(s)')
    plt.ylabel('||Error||(m)')
    plt.legend()
    plt.grid(True)
    ac.save_figure(ac.fig3, 'trajectory_tracking_e')
    

    plt.show()