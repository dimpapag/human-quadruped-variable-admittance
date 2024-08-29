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
        pT = np.array([0.0, 0.0, self.p[2]])

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

    # After the control loop finishes, plot the logged variables
    plt.figure()
    plt.plot(ac.log_t, ac.log_p[:, 0], label='Position x')
    plt.plot(ac.log_t, ac.log_pT[:, 0], label='Target Position x')
    plt.xlabel('Time')
    plt.ylabel('Position (x)')
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(ac.log_t, ac.log_v[:, 0], label='Velocity x')
    plt.xlabel('Time')
    plt.ylabel('Velocity (x)')
    plt.legend()
    plt.grid(True)

    plt.show()

  

        

        



