import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

calib_poser = ['M1', 'M2', 'M3', 'M4',
               'L1', 'L2', 'L3', 'L4',
               'R1', 'R2', 'R3', 'R4']

# Load the CSV file
df = pd.read_csv('calib_nohands_output.csv', header=None)

# Extract x, y, z coordinates
dis = df[0]
x = df[1]
y = df[2]
z = df[3]

# Create a figure for the boxplots
fig, axs = plt.subplots(4, 1, figsize=(10, 15))

# Plot boxplots for every 20 values
for i in range(0, len(df), 20):
    end = i + 20 if i + 20 < len(df) else len(df)
    axs[0].boxplot(dis[i:end] * 100, positions=[i // 20], widths=0.6)
    axs[1].boxplot(x[i:end] * 100, positions=[i // 20], widths=0.6)
    axs[2].boxplot(y[i:end] * 100, positions=[i // 20], widths=0.6)
    axs[3].boxplot(z[i:end] * 100, positions=[i // 20], widths=0.6)

# Set y-ticks from calib_poser
num_boxes = len(range(0, len(df), 20))

for i in range(4):
    axs[i].set_xticks(range(num_boxes))
    axs[i].set_xticklabels(calib_poser[:num_boxes])

# Set labels
axs[0].set_title('Distance between end effectors')
axs[0].set_ylabel('Distance in centimeters')

axs[1].set_title('X axis error')
axs[1].set_ylabel('X Error in centimeters')

axs[2].set_title('Y axis error')
axs[2].set_ylabel('Y Error in centimeters')

axs[3].set_title('Z axis error')
axs[3].set_xlabel('Position')
axs[3].set_ylabel('Z Error in centimeters')

# Show the plot
plt.tight_layout()
plt.show()

# plt.savefig('nohands_boxplots.png')
