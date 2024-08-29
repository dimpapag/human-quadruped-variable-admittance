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
        self.log_v = np.zeros(3)
        self.log_p = np.zeros(3)
        self.log_pT = np.zeros(3)
        self.log_e = np.zeros
        


        # Folder path for saving data
        self.folder_path = 'csvFolder'
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)

        # File paths for saving data
        self.position_file = os.path.join(self.folder_path, 'position_data.csv')
        self.velocity_file = os.path.join(self.folder_path, 'velocity_data.csv')

    def save_data_to_csv(self):
            with open(self.position_file, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Time', 'Position_x', 'Position_y', 'Position_z', 'Target_x', 'Target_y', 'Target_z'])
                for t, p, pT in zip(self.log_t, self.log_p, self.log_pT):
                    writer.writerow([t, p[0], p[1], p[2], pT[0], pT[1], pT[2]])

            with open(self.velocity_file, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Time', 'Velocity_x', 'Velocity_y', 'Velocity_z'])
                for t, v in zip(self.log_t, self.log_v):
                    writer.writerow([t, v[0], v[1], v[2]])    


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
        
        #Target to Reach :
        pT = np.array([3.0,4.0, self.p[2]])

        # t_now = time.time()

        t = 0.0
        total=15.0
        # actual control loop
        while not rospy.is_shutdown() and t<=total:

            t = t + self.dt
            self.log_t = np.vstack((self.log_t,t))  # logging time
            # print(time.time() - t_now)
            # t_now =  time.time()

            print(t)
            # control signal
            e = self.p - pT
            e[2] = 0.0

            v = - kp * e
            
            vb = self.R0b.T @ v #V_b = Gamma_bc * V_c

            wb = 0

            # Log variables
            self.log_v = np.vstack((self.log_v,vb))  # Log velocity
            self.log_p = np.vstack((self.log_p,self.p))    # Log position
            self.log_pT = np.vstack((self.log_pT,pT))    # Log target position

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
    plt.figure()
    plt.plot(ac.log_t, ac.log_p[:, 0], label='Position x')
    plt.plot(ac.log_t, ac.log_pT[:, 0], label='Target Position x',color='green',linestyle='--')
    plt.plot()
    plt.title("Trajectory Tracking")
    plt.xlabel('Time')
    plt.ylabel('Position (x)')
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(ac.log_t, ac.log_v[:, 0], label='Velocity x')
    plt.title("Velocity")
    plt.xlabel('Time')
    plt.ylabel('Velocity (x)')
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(ac.log_t, ac.log_e[:, 0])
    plt.title("Error")
    plt.xlabel('Time')
    plt.ylabel('Error_x')
    plt.title('Error_x')
    plt.legend()
    plt.grid(True)

    plt.show()

  

        

        



