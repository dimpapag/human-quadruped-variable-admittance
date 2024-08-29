import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.patches import Polygon, FancyArrow
from matplotlib.collections import PatchCollection
from matplotlib import cm
from scipy.spatial.transform import Rotation as R

# Define the directory containing the CSV files

#csv_file = "/home/con/catkin_ws/src/command_give/Real_Admittance_Logging/real_adm_csv_2024-08-23_11-56-50.csv"

csv_dir = "/home/con/catkin_ws/src/command_give/Real_Admittance_Logging"

# List all CSV files in the directory
csv_files = [f for f in os.listdir(csv_dir) if f.endswith('.csv')]

# Get the latest CSV file by modification time
latest_csv_file = max([os.path.join(csv_dir, f) for f in csv_files], key=os.path.getmtime)

# Read the latest CSV file into a DataFrame
df = pd.read_csv(latest_csv_file)
# Remove leading spaces from column names
df.columns = df.columns.str.strip()

fig, axs = plt.subplots(2, 1, figsize=(10, 8), num=1)  # Figure 1

# First subplot: f_m
axs[0].plot(df['time(sec)'].values, df['f_m(N)'].values, label='f_m(N)')
axs[0].set_xlabel('Time')
axs[0].set_ylabel('f_m (N)')
axs[0].legend()
axs[0].grid(True)

# Second subplot: -f_v
axs[1].plot(df['time(sec)'].values, df['-f_v'].values, label='-f_v')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('-f_v(Nm)')
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()
plt.show(block=False)

fig, axs = plt.subplots(2, 2, figsize=(12, 10),num=2)  # Figure 2
fmin = -48

# Plot f_1
axs[0, 0].plot(df['time(sec)'].values, df['f_1(N)'].values, label='f_1(N)', color='#03436A')
axs[0, 0].axhline(fmin, color='red', linestyle='--', label=f'fmin = {fmin}')
axs[0, 0].set_xlabel('Time (sec)')
axs[0, 0].set_ylabel('f_1(N)')
axs[0, 0].legend()
axs[0, 0].grid(True)

# Plot f_2
axs[0, 1].plot(df['time(sec)'].values, df['f_2(N)'].values, label='f_2(N)', color='#FF3D00')
axs[0, 1].axhline(fmin, color='red', linestyle='--', label=f'fmin = {fmin}')
axs[0, 1].set_xlabel('Time (sec)')
axs[0, 1].set_ylabel('f_2(N)')
axs[0, 1].legend()
axs[0, 1].grid(True)

# Plot f_3
axs[1, 0].plot(df['time(sec)'].values, df['f_3(N)'].values, label='f_3(N)', color='#FF70A6')
axs[1, 0].axhline(fmin, color='red', linestyle='--', label=f'fmin = {fmin}')
axs[1, 0].set_xlabel('Time (sec)')
axs[1, 0].set_ylabel('f_3(N)')
axs[1, 0].legend()
axs[1, 0].grid(True)

# Plot f_4
axs[1, 1].plot(df['time(sec)'].values, df['f_4(N)'].values, label='f_4(N)', color='#00B8D9')
axs[1, 1].axhline(fmin, color='red', linestyle='--', label=f'fmin = {fmin}')
axs[1, 1].set_xlabel('Time (sec)')
axs[1, 1].set_ylabel('f_4(N)')
axs[1, 1].legend()
axs[1, 1].grid(True)

plt.tight_layout()
plt.show(block=False)

#Power plot
plt.figure(figsize=(8, 6), num=3)  # Figure 3
plt.plot(df['time(sec)'].values, df['Power(Watt)'].values)
plt.title('Power Plot')
plt.xlabel('Time')
plt.ylabel('Power (Watt)')
plt.grid(True)
plt.show(block=False)

E=sum(df['Power(Watt)'].values) * 0.002
print("Total Energy = ",E," Joule")

fig, axs = plt.subplots(2, 1, figsize=(10, 8), num=4)  # Figure 4
axs[0].plot(df['time(sec)'].values, df['p_x(m)'].values, label='Pose X')  # Adjusted column name here
axs[0].plot(df['time(sec)'].values, df['p_y(m)'].values, label='Pose Y')  # Adjusted column name here
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Pose')
axs[0].legend()
axs[0].grid(True)

axs[1].plot(df['time(sec)'].values, df['theta_z(rad)'].values, label='Theta_Z')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Theta_Z')
axs[1].legend()
axs[1].grid(True)

plt.show(block=False)

plt.figure(figsize=(8, 6), num=5)  # Figure 3
plt.plot(df['time(sec)'].values, df['d_factor'].values)
plt.title('d factor plot')
plt.xlabel('Time')
plt.ylabel('d_factor')
plt.grid(True)
plt.show(block=True)