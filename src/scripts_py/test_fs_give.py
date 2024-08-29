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

def skew_symmetric(vector):
    if len(vector) != 3:
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
            [a/2, b/2, h],
            [a/2, -b/2, h],
            [-a/2, -b/2, h],
            [-a/2, b/2, h]
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

        self.data_storage = []
        self.store_data = self.prompt_data_storage()
        
        self.csv_filename = None  # Initialize the filename

        self.default_bias_coefficients = [
            [-0.12083, 1130.58421],
            [-0.12427, 1137.09065],
            [-0.11908, 1128.56703],
            [-0.12086, 1131.91921]
        ]

        self.bias_coefficients = self.default_bias_coefficients  # Use default biases initially

        if self.store_data and self.prompt_change_biases():
            self.collect_data_for_biases(10) #Waiting to collect data
            self.calculate_new_biases()

            print("The lines are:")
            for i, coeff in enumerate(self.bias_coefficients):
                print(f"y{i+3} = {coeff[0]}*p5 + {coeff[1]}")
            user_input = input("Are you okay with this? (Y/N): ").strip().lower()
            if user_input not in ['y', 'yes']:
                print("Using default bias coefficients.")
                self.bias_coefficients = self.default_bias_coefficients

        self.run()

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
        collected_data = np.array(self.collected_data)
        p5 = collected_data[:, 2]
        p_values = collected_data[:, [0, 1, 3, 4]]

        self.bias_coefficients = []
        for i in range(4):
            p_i = p_values[:, i]
            coeff = np.polyfit(p5, p_i, 1)  # Linear fit (1st degree polynomial)
            self.bias_coefficients.append(coeff)

        print("New bias coefficients calculated:", self.bias_coefficients)

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

                            bias3 = self.bias_coefficients[0][0] * p5 + self.bias_coefficients[0][1]
                            bias4 = self.bias_coefficients[1][0] * p5 + self.bias_coefficients[1][1]
                            bias1 = self.bias_coefficients[2][0] * p5 + self.bias_coefficients[2][1]
                            bias2 = self.bias_coefficients[3][0] * p5 + self.bias_coefficients[3][1]

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

                            msg = F_external()
                            msg.header.stamp = rospy.Time.now()
                            msg.data = F_b.tolist()
                            self.publisher.publish(msg)

                            if self.store_data:
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
            if self.store_data and self.csv_filename:
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
