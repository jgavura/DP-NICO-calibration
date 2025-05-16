import matplotlib.pyplot as plt
import numpy as np
from tablet_coords_conversion import sim2tab, pix2cm

targets, hits = [], []

file_name = f'data/exp_grid/grid_cartesian_targets.txt'
# file_name = f'testing_data/xy_to_xy/xy_to_xy_predictions.txt'
with open(file_name, 'r') as f:
    content = f.read().split('\n')[:-1]
    for j in range(len(content)):
        x, y = content[j].split(' ')[:2]
        x, y = sim2tab(float(x), float(y))
        targets.append((1920 - x, y))

file_name = f'data/exp_grid/grid_cartesian_results.txt'
# file_name = f'testing_data/xy_to_xy/xy_to_xy_testing_input.txt'
with open(file_name, 'r') as f:
    content = f.read().split('\n')[:-1]
    for j in range(len(content)):
        x, y = list(map(float, content[j].split(' ')))
        hits.append((1920 - x, y))

# print(targets)
# print(hits)

# Unpack target and attempt positions
target_x, target_y = zip(*targets)
result_x, result_y = zip(*hits)

# Calculate deviations
deviations = [np.sqrt((tx - ax)**2 + (ty - ay)**2) for (tx, ty), (ax, ay) in zip(targets, hits)]
max_deviation = np.max(deviations)
min_deviation = np.min(deviations)
avg_deviation = np.mean(deviations)

# Create plot
fig, ax = plt.subplots(figsize=(19.2, 7.8))

ax.set_axisbelow(True)

# Plot targets
plt.scatter(target_x, target_y, color='red', label='cS', marker='s', zorder=5)

# Plot attempts
plt.scatter(result_x, result_y, color='blue', label='cR', zorder=10)

# Draw lines between each target and its corresponding attempt
for (tx, ty), (ax, ay) in zip(targets, hits):
    plt.plot([tx, ax], [ty, ay], color='gray', linestyle='--', zorder=0)

# Set axis limits
plt.xlim(0, 1920)
plt.ylim(300, 1080)

# Add statistics text
stats_text = (f"2D - Max Dev: {round(pix2cm(max_deviation), 2)} cm\n"
              f"2D - Min Dev: {round(pix2cm(min_deviation), 2)} cm\n"
              f"2D - Mean Dev: {round(pix2cm(avg_deviation), 2)} cm")
plt.text(0.01, 1.08, stats_text, transform=plt.gca().transAxes,
         fontsize=15, verticalalignment='center', bbox=dict(facecolor='white', alpha=0.5))

# Add nico hand
plt.text(0.3, 1.08, "NICO\nArm", transform=plt.gca().transAxes,
         fontsize=20, verticalalignment='center', bbox=dict(facecolor='white', alpha=0.5))

# Set the font size of ticks
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)

# Add labels and legend
plt.xlabel('X-axis (px)', fontsize=20)
plt.ylabel('Y-axis (px)', fontsize=20)
plt.title('Exp - Grid - Duration: 2 s - 2025', fontsize=25, pad=20, x=0.65)
plt.legend(fontsize=15)
plt.grid(True)

# Show plot
plt.show()
# plt.savefig('plots/exp_grid_2s_2025.png')
