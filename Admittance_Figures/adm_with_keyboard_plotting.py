import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, FancyArrow
from matplotlib.collections import PatchCollection
from matplotlib import cm
from scipy.spatial.transform import Rotation as R


# Read CSV file into DataFrame
csv_file = "src/command_give/Admittance_Logging_with_Keyboard/adm_csv_2024-06-17_16-42-52.csv"
df = pd.read_csv(csv_file)
# Remove leading spaces from column names
df.columns = df.columns.str.strip()
#print(df.columns)

# Path plot
plt.figure(figsize=(8, 6), num=1)  # Figure 1
plt.plot(df['p_x(m)'].values, df['p_y(m)'].values)  # Adjusted column names here
plt.title('Path Plot')
plt.xlabel('Pose X')
plt.ylabel('Pose Y')
plt.grid(True)
plt.show(block=False)

# 2x1 subplot (Generalized position)
fig, axs = plt.subplots(2, 1, figsize=(10, 8), num=2)  # Figure 2
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

# 2x1 subplot
fig, axs = plt.subplots(2, 1, figsize=(10, 8), num=3)  # Figure 3

# First subplot: F_x and F_y
axs[0].plot(df['time(sec)'].values, df['f_x(N)'].values, label='F_x(N)')
axs[0].plot(df['time(sec)'].values, df['f_y(N)'].values, label='F_y(N)')
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Force (N)')
axs[0].legend()
axs[0].grid(True)

# Second subplot: tau_z
axs[1].plot(df['time(sec)'].values, df['tau_z(Nm)'].values, label='tau_z(Nm)')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Torque (Nm)')
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()
plt.show(block=False)

# Power plot
plt.figure(figsize=(8, 6), num=4)  # Figure 4
plt.plot(df['time(sec)'].values, df['Power(Watt)'].values)
plt.title('Power Plot')
plt.xlabel('Time')
plt.ylabel('Power (Watt)')
plt.grid(True)
plt.show()

# Special path plot with detailed robot shape
N = 10
idx = np.arange(0, len(df), len(df) // N)
fig, ax = plt.subplots(figsize=(8, 6), num=5)  # Figure 5

# Go1 robot dimensions in meters
body_length = 0.645
body_width = 0.28
head_length = 0.05  # Updated for a shorter head
head_width = 0.14
leg_length = 0.2
leg_width = 0.05

# Define the head shape more accurately as a trapezoid
head_shape = np.array([
    [body_length / 2, -head_width / 2],
    [body_length / 2 + head_length, -head_width / 4],
    [body_length / 2 + head_length, head_width / 4],
    [body_length / 2, head_width / 2]
])

# Create a pastel color gradient (rainbow)
colors = cm.rainbow(np.linspace(0, 1, N))
pastel_colors = [(r*0.7 + 0.3, g*0.7 + 0.3, b*0.7 + 0.3, a) for r, g, b, a in colors]

for i, color in zip(idx, pastel_colors):
    pose_x = df.loc[i, 'p_x(m)']  # Adjusted column name here
    pose_y = df.loc[i, 'p_y(m)']  # Adjusted column name here
    theta_Z = df.loc[i, 'theta_z(rad)']
    
    # Define the body in 2D
    body = np.array([
        [pose_x - body_length / 2, pose_y - body_width / 2],
        [pose_x + body_length / 2, pose_y - body_width / 2],
        [pose_x + body_length / 2, pose_y + body_width / 2],
        [pose_x - body_length / 2, pose_y + body_width / 2]
    ])
    
    # Define the head position in 2D
    head = head_shape + [pose_x, pose_y]

    # Combine body and head
    robot_shape = np.vstack([body, head])
    
    # Convert to 3D by adding a z-coordinate
    robot_shape_3d = np.hstack([robot_shape, np.zeros((robot_shape.shape[0], 1))])
    
    # Apply rotation
    rot = R.from_euler('z', theta_Z, degrees=False)
    robot_shape_rot_3d = rot.apply(robot_shape_3d)
    
    # Project back to 2D
    robot_shape_rot = robot_shape_rot_3d[:, :2]
    
    # Separate body and head after rotation
    body_rot = robot_shape_rot[:4]
    head_rot = robot_shape_rot[4:]
    
    # Create the polygons for body and head
    body_poly = Polygon(body_rot, closed=True, fill=True, edgecolor='black', facecolor=color, alpha=0.7)
    head_poly = Polygon(head_rot, closed=True, fill=True, edgecolor='black', facecolor=color, alpha=0.9)
    ax.add_patch(body_poly)
    ax.add_patch(head_poly)
    
    # Add a smaller arrow to indicate the robot's heading
    heading_length = 0.1  # Smaller arrow length
    heading_x = pose_x + heading_length * np.cos(theta_Z)
    heading_y = pose_y + heading_length * np.sin(theta_Z)
    ax.add_patch(FancyArrow(pose_x, pose_y, heading_x - pose_x, heading_y - pose_y, 
                            color='black', width=0.01, alpha=0.7, head_width=0.05, head_length=0.05))  # Smaller, pointier arrow

# Plot the path with a solid color
ax.plot(df['p_x(m)'].values, df['p_y(m)'].values, color='blue')  # Adjusted column names here

# Add color gradient bar
sm = plt.cm.ScalarMappable(cmap='rainbow', norm=plt.Normalize(vmin=0, vmax=N))
sm.set_array([])
cbar = plt.colorbar(sm, ax=ax)
cbar.set_label('Gradient')

ax.set_title('Special Path Plot with Detailed Robot Shape')
ax.set_xlabel('Pose X')
ax.set_ylabel('Pose Y')
ax.grid(True)
plt.show()
