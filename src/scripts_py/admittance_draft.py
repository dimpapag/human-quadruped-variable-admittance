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
         #time
        self.log_t = 0 
         #position
        self.log_p = np.zeros(3)
        self.log_Pd = np.zeros(3)
        self.log_p_ref= np.zeros(2)
         #orientation
        self.log_theta=0
        self.log_thetaT=0
        self.log_theta_ref=0
         #velocity
        self.log_vb = 0
        #self.log_vb = np.zeros(3)
        #self.log_Pd_dot = np.zeros(3)
         # position error
        self.log_e = 0
        #self.log_e = np.zeros(3)
         # orientation error
        self.log_e_o=0
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

        # projecting to the null space of z 
        Nz = np.identity(3)
        Nz[2,2] = 0 
        
        ktheta = angle_axis_vectors( np.array([1, 0, 0]),  Nz @ self.R0b[:, 0])
        
        self.theta = ktheta[2]
    
    def external_force(t, tStart, duration=5):
        if tStart < t and t < tStart + duration:
            fx = 100.0  # Specify your desired force in x direction
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

        #Controller Gains: 
        kp = 0.5
    
        # t_now = time.time()

        t = 0.0
        total=10.0
        theta0 =0
        #theta0 = self.theta
        #pulse duration 
        dur=5 # 5sec unit pulse
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
         #Desired Stiffness
        K_d=np.array([[50,0,0],
              [0,50,0],
              [0,0,10]])
        
        #Initializing matrices A,B of linear Differential equation:
        A = np.zeros((6,6))
        B = np.zeros((6,3))

        A[0:3 , 3:6] = np.identity(3)
        A[3:6 , 0:3] = - np.linalg.inv(M_d) @  K_d
        A[3:6 , 3:6] = - np.linalg.inv(M_d) @  D_d

        B[3:6 , : ] = np.linalg.inv(M_d)

        #Initial Desired Generalized Position  P_desired_(t=0) = [x_des_0; y_des_0; theta_des_0]:
        Pd0 = np.zeros(3) # array reasons 
        Pd0[0:2] = self.p[0:2]
        Pd0[2] = self.theta

        #Initial Desired Generalized Velocity (V_des_0=P_dot_des_0= [v_x_des_0_dot; v_y_des_0_dot; theta_des_0_dot])
        Pd_dot0 = np.zeros(3)

        Pd=Pd0 # for reaching that's all that's needed !
        Pd_dot=Pd_dot0

        #Trajectory to Track : Circle :
        r=0.8 # radius of circle
        numofcircles=1 # number of circles to complete one period
        freq=numofcircles/total #Hz
        omega=2*np.pi*freq #rad/s

        #M_d*d_dot_dot(t)+D_d*d_dot(t)+K_d*d(t)= Fext =>
        # Where Fext = [Fx; Fy; tau_z] , is an external force   
        # M_d, D_d, Kp are the desired impedance characteristics and
        # Where d(t) is the displacement (P_reference-P_desired)
        # when contact force Fext is applied.
        # P_reference is the position which the virtual target is going to follow. 
        #  Defining x1=d,x2=d_dot  
        #thus x1_dot = x2 , x2_dot = d_ddot = inv(M_d)*(Fext - K_d*x1 - D_d*x2)
        # x_admittance = [ x1 ; x2 ] = [d ; d_dot ]

        # Zeroing the Admittance state for python reasons
        x_ad = np.zeros(6)
        x_ad_dot = np.zeros(6)

        #Initialize states x1,x2 , x1_dot, x2_dot: 
            # We suppose that d = p_current(t=0) - p_desired(t=0) 
        x_ad[:3] = np.array([self.p[0] - Pd0[0], self.p[1] - Pd0[1], self.theta - Pd0[2]])
        x_ad[-3:] = np.zeros(3) # Initial velocities 0
        # Assuming our system starts from rest
        x_ad_dot[:3] = x_ad[-3:]
        x_ad_dot[-3:] = np.zeros(3)

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
            x_ad = x_ad + x_ad_dot * self.dt
            x_ad_dot = A @ x_ad + B @ Fext
            
    
            ##################3 Trajectory tracking ##########################

            #Desired task space trajectory, velocity. (x,y,omega_z)
            # pT=np.array([r*np.cos(omega*t),r*np.sin(omega*t),self.p[2]])
            # thetaT = theta0 +  np.pi/3 * np.sin(omega*t)

            # pT_dot=np.array([-r*omega*np.sin(omega*t),r*omega*np.cos(omega*t),0])
            # thetaT_dot= np.pi/3*omega*np.cos(omega*t)
            ### Case of target reaching p_desiered(t) = const.

            # p_des(t) = ...
            Pd= Pd0 

            # p_ref(t) = pd(t) + d(t) ,  P_ref = [position_ref ; orientation_ref]
            p_ref = Pd[0:2] + x_ad[0:2]
            theta_ref = Pd[2] + x_ad[2]
            #Same for their first order derivatives
            p_dot_ref = Pd_dot[0:2] + x_ad[3:5]
            theta_dot_ref = Pd_dot[2] + x_ad[5]

            t = t + self.dt
            self.log_t = np.vstack((self.log_t,t))  # logging time
            # print(time.time() - t_now)
            # t_now =  time.time()

            print(t)
            # Control signal 
            e = self.p[0:2] - p_ref # now  e = P - P_ref !!!
            e_o= self.theta - theta_ref        

            v[0:2] = p_dot_ref - kp * e # pd_dot-kp(e)
            v[2] = 0.0
            
            vb = self.R0b.T @ v #V_body = Gamma_body,base * V_base
            
            wb =  theta_dot_ref - kp * e_o # Omega_body

            # Log variables
             # Positions
            self.log_p = np.vstack((self.log_p,self.p))    # Log actual position
            self.log_Pd = np.vstack((self.log_Pd,Pd))    # Log desired position
            self.log_p_ref = np.vstack((self.log_p_ref,p_ref))    # Log reference position
             # Orientations
            self.log_theta= np.vstack((self.log_theta,self.theta)) # Log actual orientation
            self.log_thetaT = np.vstack((self.log_thetaT,Pd[2])) # Log desired orientation
            self.log_theta_ref = np.vstack((self.log_theta_ref,theta_ref)) # Log reference orientation
             # Velocities
            self.log_vb = np.vstack((self.log_vb,np.linalg.norm(vb)))  # Log velocity
            #self.log_vb = np.vstack((self.log_vb,vb))  # Log velocity
            #self.log_Pd_dot = np.vstack((self.log_Pd_dot,vb))    # Log target velocity
             # Errors
            #self.log_e = np.vstack((self.log_e,e)) 
            self.log_e = np.vstack((self.log_e,np.linalg.norm(e))) 
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
    ax1.plot(ac.log_t, ac.log_Pd[:, 0], label='Des Pos_x',color='red',linestyle='--')
    ax1.plot(ac.log_t, ac.log_p_ref[:, 0],  label='Ref Pos_x',color='maroon',linestyle=':')

    ax1.legend()
    ax1.grid(True)
    ax2.plot(ac.log_t,  ac.log_p[:, 1],  label='Pos_y')
    ax2.plot(ac.log_t, ac.log_Pd[:, 1], label='Des Pos_y',color='green',linestyle='--')
    ax2.plot(ac.log_t, ac.log_p_ref[:, 1],  label='Ref Pos_y',color='olive',linestyle=':')
    ax2.legend()
    ax2.grid(True)
    ax3.plot(ac.log_t, ac.log_theta,    label='Orient_z')
    ax3.plot(ac.log_t, ac.log_thetaT,   label='Des Orient_z',color='magenta',linestyle='--')
    ax3.plot(ac.log_t, ac.log_theta_ref,  label='Ref Orient_z',color='darkviolet',linestyle=':')
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