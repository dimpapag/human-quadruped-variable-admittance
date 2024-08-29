import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt

# Define the directory and file name
directory = '/home/con/catkin_ws/src/command_give/Pascal_Data'
filename = 'data_20240703_134132.csv'
filepath = os.path.join(directory, filename)

# Read the CSV file into a pandas DataFrame, skipping the header
df = pd.read_csv(filepath, header=None, skiprows=1)

# Extract values from the DataFrame
data = df.values.flatten()  # Flatten to handle potential multi-row data

# Convert the data to numeric types
data = list(map(float, data))  # Convert all elements to float

# Extract vectors from the flattened data
p3 = np.array(data[0::5])  # Every 5th element starting from index 0
p4 = np.array(data[1::5])  # Every 5th element starting from index 1
p5 = np.array(data[2::5])  # Every 5th element starting from index 2
p1 = np.array(data[3::5])  # Every 5th element starting from index 3
p2 = np.array(data[4::5])  # Every 5th element starting from index 4

# Perform linear regression using polyfit
degree = 1  # Degree of the polynomial (1 for linear)
coeffs_y3 = np.polyfit(p5, p3, degree)
coeffs_y4= np.polyfit(p5, p4, degree)
coeffs_y1 = np.polyfit(p5, p1, degree)
coeffs_y2 = np.polyfit(p5, p2, degree)

# Extract the slope and intercept from the coefficients
slope_y3, intercept_y3 = coeffs_y3
slope_y4, intercept_y4 = coeffs_y4
slope_y1, intercept_y1 = coeffs_y1
slope_y2, intercept_y2 = coeffs_y2

# Compute the equations in terms of p3
equation_y3 = f"y3 = {slope_y3}*p5 + {intercept_y3}"
equation_y4 = f"y4 = {slope_y4}*p5 + {intercept_y4}"
equation_y1 = f"y1 = {slope_y1}*p5 + {intercept_y1}"
equation_y2 = f"y2 = {slope_y2}*p5 + {intercept_y2}"

# Print the equations
print("Equations of the lines:")
print(equation_y3)
print(equation_y4)
print(equation_y1)
print(equation_y2)

# Plotting
plt.figure(figsize=(10, 6))

# Plot the lines
plt.plot(p3, slope_y3 * p3 + intercept_y3, color='red', linestyle='-', label=equation_y3)
plt.plot(p3, slope_y4 * p3 + intercept_y4, color='blue', linestyle='-', label=equation_y4)
plt.plot(p3, slope_y1 * p3 + intercept_y1, color='green', linestyle='-', label=equation_y1)
plt.plot(p3, slope_y2 * p3 + intercept_y2, color='orange', linestyle='-', label=equation_y2)

plt.xlabel('p3')
plt.ylabel('Values')
plt.title('Lines Visualization')
plt.legend()

plt.tight_layout()
plt.show()
