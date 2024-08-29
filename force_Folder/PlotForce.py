import matplotlib.pyplot as plt
import pandas as pd

# Read the log file
df = pd.read_csv("forceCSV_2024-05-15_15-44-06.csv")

# Extract data as NumPy arrays
seconds = df['time(sec)'].to_numpy()  # Assuming the column name is 'timestamp'
F_C = df[['F_C0', 'F_C1', 'F_C2', 'F_C3', 'F_C4', 'F_C5']].to_numpy()

# Calculate standard deviation for each force component
std_devs = df[['F_C0', 'F_C1', 'F_C2', 'F_C3', 'F_C4', 'F_C5']].std()
# Print out the standard deviation for each force component
for i, std_dev in enumerate(std_devs):
    print(f"Standard deviation for F_C{i}: {std_dev}")

# Plot F_C over time with random colors
plt.figure(figsize=(10, 6))
lines = []  # Store line objects for getting colors
for i in range(6):
    line, = plt.plot(seconds, F_C[:, i], label=f'F_C{i}')
    lines.append(line)  # Store line object for each plot
plt.xlabel('Time [s]')
plt.ylabel('Force Components')
plt.legend()
plt.title('F_C with respect to time')
plt.grid(True)
plt.show(block=False)  # Show the first figure without blocking

# Plot each force individually with colors from the first plot
plt.figure(figsize=(15, 10))
for i in range(6):
    plt.subplot(2, 3, i + 1)
    plt.plot(seconds, F_C[:, i], color=lines[i].get_color())  # Use color from the first plot
    plt.xlabel('Time [s]')
    plt.ylabel(f'F_C{i}')
    plt.title(f'Force Component {i}')
    plt.grid(True)

plt.tight_layout()
plt.show(block=False)  # Show the 2nd figure without blocking

# Calculate standard deviation and mean for each force component
means = []
std_devs = []
for i in range(6):
    std_dev = df[f'F_C{i}'].std()
    mean = df[f'F_C{i}'].mean()
    std_devs.append(std_dev)
    means.append(mean)
    print(f"f{i} = {mean:.2f} ± {std_dev:.2f}")

# Calculate the deadzone for each force component
num_std_devs = 1
deadzones = [num_std_devs * std_dev for std_dev in std_devs]

# Plot the deadzones for each force component individually
plt.figure(figsize=(15, 10))
for i in range(6):
    plt.subplot(2, 3, i + 1)
    plt.plot(seconds, F_C[:, i], label=f'F_C{i}')  # Plot force data
    plt.fill_between(seconds, means[i] - deadzones[i], means[i] + deadzones[i], color='gray', alpha=0.3)  # Plot deadzone
    plt.xlabel('Time [s]')
    plt.ylabel('Force Components')
    plt.legend()
    plt.title(f'F_C{i} with Deadzone')
    plt.grid(True)

plt.tight_layout()
plt.show()
