import matplotlib.pyplot as plt
import numpy as np
from tablet_coords_conversion import pix2cm
from scipy.interpolate import griddata


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

# Vytvorenie jemnejšej mriežky
xi = np.linspace(min(x), max(x), 100)  # 100 bodov na x-osi
yi = np.linspace(min(y), max(y), 100)  # 100 bodov na y-osi
X, Y = np.meshgrid(xi, yi)

# Interpolácia hodnôt na novej mriežke
Z = griddata((x, y), z, (X, Y), method='linear')  # cubic = hladká interpolácia

Z = np.clip(Z, 0, 2.8)  # Orezanie hodnôt mimo rozsah 0-2.8

fig, ax = plt.subplots(figsize=(17.5, 6.8))

# Define common contour levels
levels = np.linspace(0, 2.8, 7)  # Using the maximum range (0-4.2)

ax.contour(X, Y, Z, levels=levels, linewidths=0.5, colors='k')
cntr = ax.contourf(X, Y, Z, levels=levels, cmap="magma_r")

cbar = fig.colorbar(cntr, ax=ax, fraction=0.0185, pad=0.04)
cbar.set_label('Distance (cm)', fontsize=20)
cbar.ax.tick_params(labelsize=15)
ax.plot(x, y, 'ko', ms=3)
ax.set(xlim=(0, 1750), ylim=(400, 1080))
ax.tick_params(axis='both', which='major', labelsize=15)
ax.set_aspect('equal')
plt.title('Nonlinear model (M3) - 2D Deviation - 2 sec', fontsize=30, pad=15, x=0.72)

# Add nico hand
plt.text(0.335, 1.065, "NICO\nArm", transform=plt.gca().transAxes,
         fontsize=18, verticalalignment='center', bbox=dict(facecolor='white', alpha=0.5))

plt.tight_layout()

plt.show()
# plt.savefig('plots/m3_contour_plot.png')
# plt.savefig('plots/paper/m3_contour_plot.eps', format='eps')
