import numpy as np
import serial
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

class RealTimePlot:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, update_interval=100):
        self.serial_port = port
        self.baudrate = baudrate
        self.update_interval = update_interval

        self.open_serial_port()

        self.t = []
        self.p1 = []
        self.p2 = []
        self.p3 = []
        self.p4 = []
        self.p5 = []

        self.fig, self.axs = plt.subplots(5, 1, figsize=(10, 10))
        self.lines = [ax.plot([], [])[0] for ax in self.axs]

        for i, ax in enumerate(self.axs, start=1):
            ax.set_xlim(0, 10)
            ax.set_ylim(-10, 10)  # Adjust according to your data range
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'p{i}')

    def open_serial_port(self):
        while True:
            try:
                self.serial = serial.Serial(self.serial_port, self.baudrate)
                print(f'Serial port {self.serial_port} opened successfully.')
                break
            except serial.SerialException as e:
                print(f'Failed to open serial port {self.serial_port}: {e}')
                print('Retrying in 5 seconds...')
                time.sleep(5)

    def update_plot(self, frame):
        if self.serial.in_waiting > 0:
            data = self.serial.readline().decode("utf-8").rstrip()
            values = data.split(',')
            if len(values) >= 5:
                p3, p4, p5, p1, p2 = map(float, values[:5])

                self.t.append(time.time() % 60)
                self.p1.append(p1)
                self.p2.append(p2)
                self.p3.append(p3)
                self.p4.append(p4)
                self.p5.append(p5)

                for i, (p, line) in enumerate(zip([self.p1, self.p2, self.p3, self.p4, self.p5], self.lines)):
                    line.set_data(self.t, p)
                    self.axs[i].set_xlim(max(0, self.t[-1] - 10), self.t[-1])

        return self.lines

    def run(self):
        ani = FuncAnimation(self.fig, self.update_plot, interval=self.update_interval)
        plt.tight_layout()

        # Use PillowWriter to save the animation as a GIF file
        writer = PillowWriter(fps=10)
        ani.save('real_time_plot.gif', writer=writer)

        plt.show()

if __name__ == '__main__':
    plotter = RealTimePlot()
    plotter.run()
