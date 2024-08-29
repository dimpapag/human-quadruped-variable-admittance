#!/usr/bin/env python3

import serial
import serial.tools.list_ports

class CommandSender:

    #def __init__(self, serial_port='/dev/ttyACM0', baudrate=115200):
    def __init__(self, baudrate=115200):
    
        self.serial_port = self.find_serial_port()
        self.baudrate = baudrate
        self.serial = None

    def find_serial_port(self):
        # Find all available serial ports
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if 'ttyACM' in port.device:
                return port.device
        raise serial.SerialException("No suitable serial port found.")


    def check_serial_connection(self):
        try:
            self.serial = serial.Serial(self.serial_port, self.baudrate)
            print(f'Serial port {self.serial_port} connection successful.')
            self.serial.close()  # Close the serial port immediately after successful connection
            return True
        except serial.SerialException as e:
            print(f'Failed to open serial port {self.serial_port}: {e}')
            return False

    def open_serial_port(self):
        try:
            self.serial = serial.Serial(self.serial_port, self.baudrate)
            print(f'Serial port {self.serial_port} opened successfully for command sending.')
        except serial.SerialException as e:
            print(f'Failed to open serial port {self.serial_port}: {e}')

    def send_command(self, command):
        try:
            if not self.serial or not self.serial.is_open:
                self.open_serial_port()
            self.serial.write((command + '\n').encode())
            print(f'Sent command: {command}')
        except serial.SerialException as e:
            print(f'Failed to send command {command}: {e}')
        finally:
            if self.serial and self.serial.is_open:
                self.serial.close()
                print(f'Serial port {self.serial_port} closed.')

if __name__ == '__main__':
    sender = CommandSender()

    # Check initial serial connection
    if not sender.check_serial_connection():
        print("Exiting program.")
        exit(1)

    try:
        while True:
            user_input = input("Enter command ('11' for on, '0' for off, 'p' to quit): ").strip()

            if user_input == 'p':
                break

            if user_input == '11' or user_input == '0':
                sender.send_command(user_input)
            else:
                print("Invalid command. Please enter '11', '0', or 'p'.")

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        if sender.serial and sender.serial.is_open:
            sender.serial.close()
            print(f'Serial port {sender.serial_port} closed.')
