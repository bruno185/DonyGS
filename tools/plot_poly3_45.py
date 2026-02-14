import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Face 45 points
poly45 = [(233,114),(98,88),(105,89),(239,115)]
# Face 3 points
poly3 = [(218,108),(218,114),(206,112),(206,105)]

plt.figure(figsize=(6,5), dpi=150)

# Plot Face 45
xs45, ys45 = zip(*poly45)
plt.plot(list(xs45)+[xs45[0]], list(ys45)+[ys45[0]], color='tab:blue', linewidth=2, label='Face 45')
plt.scatter(xs45, ys45, color='tab:blue', zorder=4)
for i, (x,y) in enumerate(poly45):
    plt.text(x + 3, y + 3, f'45:{i}', color='tab:blue', fontsize=10, zorder=5)

# Plot Face 3
xs3, ys3 = zip(*poly3)
plt.plot(list(xs3)+[xs3[0]], list(ys3)+[ys3[0]], color='tab:green', linewidth=2, label='Face 3')
plt.scatter(xs3, ys3, color='tab:green', zorder=4)
for i, (x,y) in enumerate(poly3):
    plt.text(x + 3, y + 3, f'3:{i}', color='tab:green', fontsize=10, zorder=5)

# Annotations: show coords optionally (commented out)
# for (x,y) in poly45+poly3:
#     plt.text(x, y-6, f'({x},{y})', fontsize=8, color='gray')

# Viewport and styling
all_x = xs45 + xs3
all_y = ys45 + ys3
minx, maxx = min(all_x), max(all_x)
miny, maxy = min(all_y), max(all_y)
padx = max(6, (maxx - minx) * 0.12)
pady = max(6, (maxy - miny) * 0.12)
plt.xlim(minx - padx, maxx + padx)
plt.ylim(miny - pady, maxy + pady)
plt.gca().invert_yaxis()  # match screen coordinates
plt.gca().set_aspect('equal', adjustable='box')
plt.title('Faces 3 (green) and 45 (blue)')
plt.xlabel('x2d')
plt.ylabel('y2d')
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend()

out = 'poly3_45.png'
plt.savefig(out, bbox_inches='tight')
print('Saved', out)
