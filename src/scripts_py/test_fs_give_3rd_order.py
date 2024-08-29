#!/usr/bin/env python3

import os
import serial
import rospy
import numpy as np
from std_msgs.msg import String
from datetime import datetime
from command_give.msg import F_external  # Adjust according to your message definition
import csv
import time
import matplotlib.pyplot as plt

def skew_symmetric(vector):
    if (len(vector) != 3):
        raise ValueError("Input vector must have exactly 3 elements.")
    skew_matrix = np.array([
        [0, -vector[2], vector[1]],
        [vector[2], 0, -vector[0]],
        [-vector[1], vector[0], 0]
    ])
    return skew_matrix

class SerialNode:
    def __init__(self):
        rospy.init_node('serial_node_teensy', anonymous=True)
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baudrate = 115200
        self.publisher = rospy.Publisher('F_external', F_external, queue_size=10)
        self.rate = rospy.Rate(500)  # 500Hz publishing rate

        a, b, h = 0.025, 0.028, -0.010  # Replace with actual values
        self.positions = [
            [a/2, b/2, h],    # Position for P1 (+,+)
            [a/2, -b/2, h],   # Position for P2 (+,-)
            [-a/2, -b/2, h],  # Position for P3 (-,-)
            [-a/2, b/2, h]    # Position for P4 (-,+)
        ]

        self.gamma_matrices = [np.block([[np.eye(3), np.zeros((3, 3))], [skew_symmetric(pos), np.eye(3)]]) for pos in self.positions]

        self.R_bs = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])

        self.p_bs = np.array([0.24, 0.0, 0.11])
        self.G_bs = np.block([
            [self.R_bs, np.zeros((3, 3))],
            [np.dot(skew_symmetric(self.p_bs), self.R_bs), self.R_bs]
        ])

        self.open_serial_port()

        self.data_storage = False  # Default to not storing data
        self.prompt_initial_settings()

        self.csv_filename = None  # Initialize the filename

        self.default_bias_coefficients = [
            [-0.12083, 1130.58421],  # For P3
            [-0.12427, 1137.09065],  # For P4
            [-0.11908, 1128.56703],  # For P1
            [-0.12086, 1131.91921]   # For P2
        ]

        self.bias_coefficients = self.default_bias_coefficients  # Use default biases initially

        if self.prompt_change_biases():
            self.collect_data_for_biases(10)  # Collect data for X seconds
            if self.calculate_new_biases():
                user_input = input("Do you accept these bias coefficients? (Y/N): ").strip().lower()
                if user_input not in ['y', 'yes']:
                    print("Using default bias coefficients.")
                    self.bias_coefficients = self.default_bias_coefficients

        self.run()

    def prompt_initial_settings(self):
        if self.prompt_data_storage():
            self.data_storage = True

    def prompt_data_storage(self):
        user_input = input("Do you want to store the data? (Y/N): ").strip().lower()
        return user_input in ['y', 'yes']

    def prompt_change_biases(self):
        user_input = input("Do you want to change biases? (Y/N): ").strip().lower()
        return user_input in ['y', 'yes']

    def collect_data_for_biases(self, t_data):
        print(f"Collecting data for {t_data} seconds...")
        start_time = time.time()
        self.collected_data = []

        while time.time() - start_time < t_data:
            if self.serial.in_waiting > 0:
                data = self.serial.readline().decode("utf-8").rstrip()
                values = data.split(',')
                if len(values) >= 5:
                    p3, p4, p5, p1, p2 = map(float, [values[0], values[1], values[2], values[3], values[4]])
                    self.collected_data.append([p3, p4, p5, p1, p2])

    def calculate_new_biases(self):
        if not self.collected_data:
            print("No data collected for bias adjustment.")
            return False

        collected_data = np.array(self.collected_data)
        p5 = collected_data[:, 2]
        p_values = collected_data[:, [0, 1, 3, 4]]

        self.bias_coefficients = []
        bias_prompt_lines = []

        for i in range(4):
            p_i = p_values[:, i]
            coeff = np.polyfit(p5, p_i, 3)  # 3rd degree polynomial fit
            self.bias_coefficients.append(coeff)
            bias_prompt_lines.append(f"y{i+1} = {coeff[0]:.4f} * p5^3 + {coeff[1]:.4f} * p5^2 + {coeff[2]:.4f} * p5 + {coeff[3]:.4f}")

        # Print biases in requested format
        print("New bias coefficients calculated:")
        for line in bias_prompt_lines:
            print(line)

        self.plot_bias_coefficients(p5, p_values)

        return True

    def plot_bias_coefficients(self, p5, p_values):
        plt.figure(figsize=(12, 8))
        
        colors = ['blue', 'green', 'orange', 'purple']
        
        for i in range(4):
            plt.subplot(2, 2, i + 1)
            p_i = p_values[:, i]
            coeff = self.bias_coefficients[i]
            plt.scatter(p5, p_i, label='Collected Data', color=colors[i])
            p5_sorted = np.sort(p5)
            plt.plot(p5_sorted, np.polyval(coeff, p5_sorted), color='red', label=f'Fit Line: y{i+1} = {coeff[0]:.4f} * p5^3 + {coeff[1]:.4f} * p5^2 + {coeff[2]:.4f} * p5 + {coeff[3]:.4f}')
            plt.xlabel('p5')
            plt.ylabel(f'p{i+1}')
            plt.legend()
            plt.title(f'Bias Coefficient Calculation for p{i+1}')
        
        plt.tight_layout()
        plt.show()

    def open_serial_port(self):
        while not rospy.is_shutdown():
            try:
                self.serial = serial.Serial(self.serial_port, self.baudrate)
                rospy.loginfo(f'Serial port {self.serial_port} opened successfully.')
                break
            except serial.SerialException as e:
                rospy.logwarn(f'Failed to open serial port {self.serial_port}: {e}')
                rospy.logwarn('Retrying in 5 seconds...')
                rospy.sleep(5)

    def run(self):
        try:
            while not rospy.is_shutdown():
                try:
                    if not self.serial.is_open:
                        self.open_serial_port()

                    if self.serial.in_waiting > 0:
                        data = self.serial.readline().decode("utf-8").rstrip()
                        rospy.loginfo(f'Received data: {data}')

                        values = data.split(',')
                        if len(values) >= 5:
                            p3, p4, p5, p1, p2 = map(float, [values[0], values[1], values[2], values[3], values[4]])
                            bias3 = np.polyval(self.bias_coefficients[0], p5)
                            bias4 = np.polyval(self.bias_coefficients[1], p5)
                            bias1 = np.polyval(self.bias_coefficients[2], p5)
                            bias2 = np.polyval(self.bias_coefficients[3], p5)

                            p_values = [p3 - bias3, p4 - bias4, p1 - bias1, p2 - bias2]

                            P3 = [0, 0, p_values[0], 0, 0, 0]
                            P4 = [0, 0, p_values[1], 0, 0, 0]
                            P1 = [0, 0, p_values[2], 0, 0, 0]
                            P2 = [0, 0, p_values[3], 0, 0, 0]
                            
                            F1 = np.dot(self.gamma_matrices[0], P1)
                            F2 = np.dot(self.gamma_matrices[1], P2)
                            F3 = np.dot(self.gamma_matrices[2], P3)
                            F4 = np.dot(self.gamma_matrices[3], P4)

                            F_s = F1 + F2 + F3 + F4

                            F_b = np.dot(self.G_bs, F_s)
                            combined_forces = np.concatenate((F_s, F_b))

                            msg = F_external()
                            msg.header.stamp = rospy.Time.now()
                            msg.data = combined_forces.tolist()
                            #msg.data = F_b.tolist()
                            self.publisher.publish(msg)

                            if self.data_storage:
                                self.store_data_to_csv(p3, p4, p5, p1, p2)

                    self.rate.sleep()

                except serial.SerialException as e:
                    rospy.logwarn(f'Serial port error: {e}')
                    rospy.logwarn('Attempting to reopen serial port...')
                    self.open_serial_port()
                    rospy.sleep(1)
                except Exception as ex:
                    rospy.logerr(f'Exception occurred: {ex}')

        finally:
            if self.data_storage and self.csv_filename:
                rospy.loginfo(f"Data stored in {self.csv_filename}")

    def store_data_to_csv(self, p3, p4, p5, p1, p2):
        if not self.csv_filename:
            directory = "/home/con/catkin_ws/src/command_give/Pascal_Data"
            if not os.path.exists(directory):
                os.makedirs(directory)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_filename = f"{directory}/data_{timestamp}.csv"
            with open(self.csv_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['p3', 'p4', 'p5', 'p1', 'p2'])

        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([p3, p4, p5, p1, p2])

if __name__ == '__main__':
    try:
        SerialNode()
    except rospy.ROSInterruptException:
        pass
