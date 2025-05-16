import matplotlib.pyplot as plt
from tablet_coords_conversion import sim2tab, pix2cm
import numpy as np
import matplotlib.image as mpimg
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from data_handling import load_data

data, targets, hits = [], [], []

no_nn_x, no_nn_y = load_data('xy')
for i in range(len(no_nn_y)):
    x, y = no_nn_y[i]
    no_nn_y[i] = sim2tab(x, y)
targets.append(no_nn_y)
hits.append(no_nn_x)

for target_form in ['xy', 'xyz']:
    targets.append([])
    hits.append([])

    for i in range(1, 11):
        file_name = f'testing_data/accuracy_exp/targets/targets{i}.txt'
        with open(file_name, 'r') as f:
            content = f.read().split('\n')[:-1]
            for j in range(len(content)):
                x, y = list(map(float, content[j].split(' ')))
                targets[-1].append((1920 - x, y))

        file_name = f'testing_data/accuracy_exp/xy_to_{target_form}/xy_to_{target_form}_results{i}.txt'
        with open(file_name, 'r') as f:
            content = f.read().split('\n')[:-1]
            for j in range(len(content)):
                x, y = list(map(float, content[j].split(' ')))
                hits[-1].append((1920 - x, y))

# for i in range(2):
#     print(targets[i])
#     print(hits[i])

for i in range(3):
    data.append([])

    for j in range(len(targets[i])):
        x1, y1 = targets[i][j]
        x2, y2 = hits[i][j]
        dev = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        data[-1].append(pix2cm(dev))

# for i in range(2):
#     print(np.max(data[i]))
#     print(np.min(data[i]))
#     print(np.mean(data[i]))

# Setting the size of the figure
fig, ax = plt.subplots(figsize=(7, 8))

# Creating the box plot with patch_artist=True to allow changing colors
box = ax.boxplot(data, patch_artist=True, flierprops=dict(markerfacecolor='lightblue', marker='o', markersize=6))

# Changing the color of the boxes
for patch in box['boxes']:
    patch.set_facecolor('lightblue')  # Change to desired color

# Adding title and labels
plt.title('Rnd Targets with/out NN Correction\n2D Deviation - 2 sec', fontsize=20, pad=20)
plt.ylabel('Distance (cm)', fontsize=17)

# Drawing a vertical line in the middle of the plot
# plt.axvline(x=5.5, color='gray', linestyle='--')

# ax.set_axisbelow(True)

# Adding horizontal gridlines
ax.yaxis.grid(color='gray', linestyle='dashed', zorder=0)

# Setting custom x tick labels
plt.xticks([i for i in range(1, 4)], [
    'M1',
    'M2',
    'M3'
])

# Set the font size of ticks
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)

# Displaying the plot
plt.show()
# plt.savefig('plots/random_wo_nn_boxplot.png')
# plt.savefig('plots/paper/random_wo_nn_boxplot_slim.eps', format='eps')
