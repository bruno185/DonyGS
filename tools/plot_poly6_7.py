import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Face 6 points
poly6 = [(191,106),(206,105),(206,112)]
# Face 7 points
poly7 = [(191,106),(212,101),(206,105)]

plt.figure(figsize=(6,5), dpi=150)

# Plot Face 6
xs6, ys6 = zip(*poly6)
plt.plot(list(xs6)+[xs6[0]], list(ys6)+[ys6[0]], color='tab:orange', linewidth=2, label='Face 6')
plt.scatter(xs6, ys6, color='tab:orange', zorder=4)
for i, (x,y) in enumerate(poly6):
    plt.text(x + 3, y + 3, f'6:{i}', color='tab:orange', fontsize=10, zorder=5)

# Plot Face 7
xs7, ys7 = zip(*poly7)
plt.plot(list(xs7)+[xs7[0]], list(ys7)+[ys7[0]], color='tab:red', linewidth=2, label='Face 7')
plt.scatter(xs7, ys7, color='tab:red', zorder=4)
for i, (x,y) in enumerate(poly7):
    plt.text(x + 3, y + 3, f'7:{i}', color='tab:red', fontsize=10, zorder=5)

# Viewport and styling
all_x = xs6 + xs7
all_y = ys6 + ys7
minx, maxx = min(all_x), max(all_x)
miny, maxy = min(all_y), max(all_y)
padx = max(6, (maxx - minx) * 0.12)
pady = max(6, (maxy - miny) * 0.12)
plt.xlim(minx - padx, maxx + padx)
plt.ylim(miny - pady, maxy + pady)
plt.gca().invert_yaxis()  # match screen coordinates
plt.gca().set_aspect('equal', adjustable='box')
plt.title('Faces 6 (orange) and 7 (red)')
plt.xlabel('x2d')
plt.ylabel('y2d')
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend()

out = 'poly6_7.png'
plt.savefig(out, bbox_inches='tight')
print('Saved', out)
