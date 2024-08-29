import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt

# Define the directory and file name
directory = '/home/con/catkin_ws/src/command_give/Pascal_Data'
filename = 'data_20240705_164558.csv'
filepath = os.path.join(directory, filename)

# Read the CSV file into a pandas DataFrame, including the header
df = pd.read_csv(filepath)

# Extract values from the DataFrame
t = df['t'].values
p3 = df['p3'].values
p4 = df['p4'].values
p5 = df['p5'].values
p1 = df['p1'].values
p2 = df['p2'].values

# Perform linear regression using polyfit
degree = 1  # Degree of the polynomial (1 for linear)
coeffs_y3 = np.polyfit(p5, p3, degree)
coeffs_y4 = np.polyfit(p5, p4, degree)
coeffs_y1 = np.polyfit(p5, p1, degree)
coeffs_y2 = np.polyfit(p5, p2, degree)

# Extract the slope and intercept from the coefficients
slope_y3, intercept_y3 = coeffs_y3
slope_y4, intercept_y4 = coeffs_y4
slope_y1, intercept_y1 = coeffs_y1
slope_y2, intercept_y2 = coeffs_y2

# Compute the equations in terms of p3
equation_y3 = f"y3 = {slope_y3:.4f} * p5 + {intercept_y3:.4f}"
equation_y4 = f"y4 = {slope_y4:.4f} * p5 + {intercept_y4:.4f}"
equation_y1 = f"y1 = {slope_y1:.4f} * p5 + {intercept_y1:.4f}"
equation_y2 = f"y2 = {slope_y2:.4f} * p5 + {intercept_y2:.4f}"

# Print the equations
print("Equations of the lines:")
print(equation_y3)
print(equation_y4)
print(equation_y1)
print(equation_y2)

# Plotting the lines
plt.figure(figsize=(10, 6))

plt.plot(p5, slope_y3 * p5 + intercept_y3, color='red', linestyle='-', label=equation_y3)
plt.plot(p5, slope_y4 * p5 + intercept_y4, color='blue', linestyle='-', label=equation_y4)
plt.plot(p5, slope_y1 * p5 + intercept_y1, color='green', linestyle='-', label=equation_y1)
plt.plot(p5, slope_y2 * p5 + intercept_y2, color='orange', linestyle='-', label=equation_y2)

plt.xlabel('p5')
plt.ylabel('Values')
plt.title('Lines Visualization')
plt.legend()

plt.tight_layout()
plt.show()

# Plotting each p with respect to time
plt.figure(figsize=(12, 10))

plt.subplot(5, 1, 1)
plt.plot(t, p3, label='p3', color='red')
plt.xlabel('Time (s)')
plt.ylabel('p3')
plt.title('p3 vs Time')
plt.legend()

plt.subplot(5, 1, 2)
plt.plot(t, p4, label='p4', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('p4')
plt.title('p4 vs Time')
plt.legend()

plt.subplot(5, 1, 3)
plt.plot(t, p5, label='p5', color='green')
plt.xlabel('Time (s)')
plt.ylabel('p5')
plt.title('p5 vs Time')
plt.legend()

plt.subplot(5, 1, 4)
plt.plot(t, p1, label='p1', color='orange')
plt.xlabel('Time (s)')
plt.ylabel('p1')
plt.title('p1 vs Time')
plt.legend()

plt.subplot(5, 1, 5)
plt.plot(t, p2, label='p2', color='purple')
plt.xlabel('Time (s)')
plt.ylabel('p2')
plt.title('p2 vs Time')
plt.legend()

plt.tight_layout()
plt.show()
