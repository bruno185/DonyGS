import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Points for Face 3
pts = [(218,108),(218,114),(206,112),(206,105)]
xs, ys = zip(*pts)

plt.figure(figsize=(6,4), dpi=150)
# Draw polygon edges and markers, closing the loop
plt.plot(list(xs)+[xs[0]], list(ys)+[ys[0]], '-k', linewidth=2)
plt.scatter(xs, ys, color='blue', zorder=3)

# Annotate indices next to each point
for i, (x, y) in enumerate(pts):
    plt.text(x + 3, y + 3, str(i), color='red', fontsize=12, zorder=4)

# Fit view with small margin
minx, maxx = min(xs), max(xs)
miny, maxy = min(ys), max(ys)
padx = max(6, (maxx - minx) * 0.1)
pady = max(6, (maxy - miny) * 0.1)
plt.xlim(minx - padx, maxx + padx)
plt.ylim(miny - pady, maxy + pady)

# Use screen-like coordinates (y down) to match x2d/y2d conventions
plt.gca().invert_yaxis()
plt.gca().set_aspect('equal', adjustable='box')
plt.title('Polygon (Face 3)')
plt.xlabel('x2d')
plt.ylabel('y2d')
plt.grid(True, linestyle='--', alpha=0.3)

out = 'poly3.png'
plt.savefig(out, bbox_inches='tight')
print('Saved', out)
