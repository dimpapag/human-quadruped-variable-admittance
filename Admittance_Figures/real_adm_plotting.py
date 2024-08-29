import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.patches import Polygon, FancyArrow
from matplotlib.collections import PatchCollection
from matplotlib import cm
from scipy.spatial.transform import Rotation as R

# Read CSV file into DataFrame
csv_file = "/home/con/catkin_ws/src/command_give/Real_Admittance_Logging/real_adm_csv_2024-08-23_17-10-10_fvEqual1.csv"
df = pd.read_csv(csv_file)
# Remove leading spaces from column names
df.columns = df.columns.str.strip()

# Subplot for f_1, f_2, f_3, and f_4 with respect to time
def fig1(df):
    fig, axs = plt.subplots(2, 2, figsize=(12, 10),num=1)  # Figure 1
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

fig1(df)

# Additional figure for -f_v with respect to time
def fig2(df):
    fig, axs = plt.subplots(2, 1, figsize=(10, 8), num=2)  # Figure 1

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

fig2(df)

# Additional figure for d_factor with respect to time
def fig3(df):
    plt.figure(figsize=(10, 6), num=3) # Figure 3
    plt.plot(df['time(sec)'].values, df['d_factor'].values, label='d_factor')
    plt.xlabel('Time (sec)')
    plt.ylabel('d')
    plt.title('Daming Coeff - Time')
    plt.grid(True)
    plt.show(block=False)

fig3(df)

# Path plot ( px-py)
def fig4(df):
    plt.figure(figsize=(8, 6), num=4)  # Figure 4
    plt.plot(df['p_x(m)'].values, df['p_y(m)'].values)  # Adjusted column names here
    plt.title('Path Plot')
    plt.xlabel('Pose X')
    plt.ylabel('Pose Y')
    plt.grid(True)
    plt.show(block=False)

fig4(df)

# Omega_Z w/respect to time
def fig5(df):
    plt.figure(figsize=(8, 6), num=5)  # Figure 5
    plt.plot( df['time(sec)'].values, df['omega_z(rad/s)'].values)  # Adjusted column names here
    plt.title('Omega')
    plt.xlabel('t')
    plt.ylabel('omega_z (rad/s)')
    plt.grid(True)
    plt.show(block=False)

fig5(df)

# 2x1 subplot (Generalized position) px-t py-t theta_z-t
def fig6(df):
    fig, axs = plt.subplots(2, 1, figsize=(10, 8), num=6)  # Figure 6
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

fig6(df)

# 2x1 subplot (Force and Torque) f_x-t f_y-t tau_z-t
def fig7(df):
    fig, axs = plt.subplots(2, 1, figsize=(10, 8), num=7)  # Figure 7

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

fig7(df)

# Power plot
def fig8(df):
    plt.figure(figsize=(8, 6), num=8)  # Figure 8
    plt.plot(df['time(sec)'].values, df['Power(Watt)'].values)
    plt.title('Power Plot')
    plt.xlabel('Time')
    plt.ylabel('Power (Watt)')
    plt.grid(True)
    plt.show(block=False)

fig8(df)

# Special path plot with detailed robot shape
N = 10
idx = np.arange(0, len(df), len(df) // N)
fig, ax = plt.subplots(figsize=(12, 8), num=9)  # Figure 9

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

# Get time values
time_values = df['time(sec)'].values
time_max = np.max(time_values)
time_min = np.min(time_values)

# Create a colormap for the time gradient
norm = plt.Normalize(time_min, time_max)
colors = cm.rainbow(norm(time_values))

# Use a subset of colors for the robot positions
pastel_colors = [(r*0.7 + 0.3, g*0.7 + 0.3, b*0.7 + 0.3, a) for r, g, b, a in colors[idx]]

for i, color in zip(idx, pastel_colors):
    pose_x = df.loc[i, 'p_x(m)']
    pose_y = df.loc[i, 'p_y(m)']
    theta_Z = df.loc[i, 'theta_z(rad)']

    # Define the body in 2D
    body = np.array([
        [-body_length / 2, -body_width / 2],
        [body_length / 2, -body_width / 2],
        [body_length / 2, body_width / 2],
        [-body_length / 2, body_width / 2]
    ])

    head = head_shape

    # Rotate the shape
    rot = R.from_euler('z', theta_Z, degrees=False).as_matrix()[:2, :2]
    body_rot = body @ rot.T
    head_rot = head @ rot.T

    # Translate to the correct position
    body_rot[:, 0] += pose_x
    body_rot[:, 1] += pose_y
    head_rot[:, 0] += pose_x
    head_rot[:, 1] += pose_y

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
                            color='black', width=0.01, alpha=0.7, head_width=0.05, head_length=0.05))  # Pointier arrow

# Plot the path with a solid color
ax.plot(df['p_x(m)'].values, df['p_y(m)'].values, color='blue')

# Add color gradient bar
sm = plt.cm.ScalarMappable(cmap='rainbow', norm=plt.Normalize(vmin=time_min, vmax=time_max))
sm.set_array([])
cbar = plt.colorbar(sm, ax=ax)
cbar.set_label('Time (sec)')

# Add ticks to the colorbar at the time instances
ticks = time_values[idx]
cbar.set_ticks(ticks)
cbar.set_ticklabels([f'{tick:.2f}' for tick in ticks])

# Set the aspect of the plot to be equal
ax.set_aspect('equal', 'box')

ax.set_title('Special Path Plot with Detailed Robot Shape')
ax.set_xlabel('Pose X')
ax.set_ylabel('Pose Y')
ax.grid(True)
plt.show(block=False) # I can put block false here 

#HERE PUT THE NEW FIGURE !!!
# Create interactive plot figure
fig_interactive, ax_interactive = plt.subplots(figsize=(12, 8), num=10)  # Figure 10

# Define initial time step index
initial_index = 0

# Plot the path with a solid color for initial time step
ax_interactive.plot(df['p_x(m)'].values, df['p_y(m)'].values, color='blue')

# Define the limits for the axes based on the overall path range
x_min, x_max = df['p_x(m)'].min()-1, df['p_x(m)'].max()+1
y_min, y_max = df['p_y(m)'].min()-0.5, df['p_y(m)'].max()+0.5

# Function to update plot based on slider value
def update_plot(val):
    idx = int(slider.val)
    ax_interactive.clear()
    
    # Plot the path with a solid color
    ax_interactive.plot(df['p_x(m)'].values, df['p_y(m)'].values, color='blue')

    # Go1 robot dimensions in meters
    body_length = 0.645
    body_width = 0.28
    head_length = 0.05  # Updated for a shorter head
    head_width = 0.14

    # Define the head shape more accurately as a trapezoid
    head_shape = np.array([
        [body_length / 2, -head_width / 2],
        [body_length / 2 + head_length, -head_width / 4],
        [body_length / 2 + head_length, head_width / 4],
        [body_length / 2, head_width / 2]
    ])

    pose_x = df.loc[idx, 'p_x(m)']
    pose_y = df.loc[idx, 'p_y(m)']
    theta_Z = df.loc[idx, 'theta_z(rad)']

    # Define the body in 2D
    body = np.array([
        [-body_length / 2, -body_width / 2],
        [body_length / 2, -body_width / 2],
        [body_length / 2, body_width / 2],
        [-body_length / 2, body_width / 2]
    ])

    head = head_shape

    # Rotate the shape
    rot = R.from_euler('z', theta_Z, degrees=False).as_matrix()[:2, :2]
    body_rot = body @ rot.T
    head_rot = head @ rot.T

    # Translate to the correct position
    body_rot[:, 0] += pose_x
    body_rot[:, 1] += pose_y
    head_rot[:, 0] += pose_x
    head_rot[:, 1] += pose_y

    # Create the polygons for body and head
    body_poly = Polygon(body_rot, closed=True, fill=True, edgecolor='black', facecolor='red', alpha=0.7)
    head_poly = Polygon(head_rot, closed=True, fill=True, edgecolor='black', facecolor='blue', alpha=0.9)
    ax_interactive.add_patch(body_poly)
    ax_interactive.add_patch(head_poly)

    # Add a smaller arrow to indicate the robot's heading
    heading_length = 0.1  # Smaller arrow length
    heading_x = pose_x + heading_length * np.cos(theta_Z)
    heading_y = pose_y + heading_length * np.sin(theta_Z)
    ax_interactive.add_patch(FancyArrow(pose_x, pose_y, heading_x - pose_x, heading_y - pose_y,
                                        color='black', width=0.01, alpha=0.7, head_width=0.05, head_length=0.05))

    # Set the fixed limits for the axes
    ax_interactive.set_xlim(x_min, x_max)
    ax_interactive.set_ylim(y_min, y_max)

    ax_interactive.set_aspect('equal', 'box')
    ax_interactive.set_title('Interactive Robot Path')
    ax_interactive.set_xlabel('Pose X')
    ax_interactive.set_ylabel('Pose Y')
    ax_interactive.grid(True)
    fig_interactive.canvas.draw_idle()

# Create a slider widget
slider_ax = plt.axes([0.1, 0.02, 0.8, 0.05])
slider = Slider(slider_ax, 'Time Step', 0, len(df)-1, valinit=initial_index, valstep=1)
slider.on_changed(update_plot)

# Initialize the plot with the initial time step
update_plot(initial_index)

plt.show()