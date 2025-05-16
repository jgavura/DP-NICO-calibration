import matplotlib.pyplot as plt
import numpy as np
from tablet_coords_conversion import pix2cm


x, y, z = [], [], []
for i in range(1, 11):
    with (open(f'testing_data/accuracy_exp/targets/targets{i}.txt', 'r') as f_targets,
          open(f'testing_data/accuracy_exp/xy_to_xyz/xy_to_xyz_results{i}.txt', 'r') as f_hits):
        targets, hits = f_targets.read().split('\n'), f_hits.read().split('\n')
        for j in range(len(targets)):
            if targets[j] == '' or hits[j] == '':
                continue
            x1, y1 = tuple(map(float, targets[j].split(' ')))
            x2, y2 = tuple(map(float, hits[j].split(' ')))
            x.append(1920 - x1)
            y.append(y1)
            z.append(pix2cm(np.sqrt((x1 - x2)**2 + (y1 - y2)**2)))

fig, ax = plt.subplots(figsize=(17.5, 6.8))

# Define common contour levels
levels = np.linspace(0, 2.8, 7)  # Using the maximum range (0-4.2)

ax.tricontour(x, y, z, levels=levels, linewidths=0.5, colors='k')
cntr = ax.tricontourf(x, y, z, levels=levels, cmap="magma_r")

cbar = fig.colorbar(cntr, ax=ax, fraction=0.046, pad=0.04)
cbar.set_label('Distance (cm)', fontsize=15)
ax.plot(x, y, 'ko', ms=3)
ax.set(xlim=(0, 1750), ylim=(400, 1080))
ax.set_aspect('equal')
plt.title('Nonlinear model (M3) - 2D Deviation - 2 sec', fontsize=30, pad=15)

plt.tight_layout()

plt.show()
# plt.savefig('plots/paper/m3_tricontour_plot.eps', format='eps')
