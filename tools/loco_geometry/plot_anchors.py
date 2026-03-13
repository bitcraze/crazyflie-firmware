import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Anchor data: ID, x, y, z
anchors = [
    (0, 2.535, 4.392, 0.296),
    (1, 3.245, -3.853, 0.865),
    (2, -2.759, -4.171, 0.870),
    (3, -3.660, 3.789, 0.274),
    (4, 2.780, 3.821, 2.870),
    (5, 3.181, -3.102, 3.545),
    (6, -2.736, -3.544, 3.610),
    (7, -3.591, 3.322, 2.750),
    (8, -0.097, 0.208, 3.174),
]

# Real anchor positions (ID, x, y, z)
real_anchors = [
    (0, 3.0, 4.079999923706055, 0.20000000298023224),
    (1, 3.0, -4.079999923706055, 0.20000000298023224),
    (2, -3.0, -4.079999923706055, 0.20000000298023224),
    (3, -3.0, 4.079999923706055, 0.20000000298023224),
    (4, 3.0, 3.7100000381469727, 3.180000066757202),
    (5, 3.0, -3.7300000190734863, 3.180000066757202),
    (6, -3.0, -3.7300000190734863, 3.180000066757202),
    (7, -3.0, 3.7100000381469727, 3.180000066757202),
    (8, 0.019999999552965164, -0.019999999552965164, 3.180000066757202),
]

ids = [a[0] for a in anchors]
x = [a[1] for a in anchors]
y = [a[2] for a in anchors]
z = [a[3] for a in anchors]

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot anchor positions
ax.scatter(x, y, z, c='b', marker='o', s=60, label='Anchors')

# Annotate each anchor with its ID
for i, (xi, yi, zi) in enumerate(zip(x, y, z)):
    ax.text(xi, yi, zi, str(ids[i]), color='black', fontsize=10)

# Print pairwise distances for estimated anchors
from itertools import combinations
def print_distances(anchor_list, label):
    print(f"\nPairwise distances for {label}:")
    for (id1, x1, y1, z1), (id2, x2, y2, z2) in combinations(anchor_list, 2):
        dist = ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2) ** 0.5
        print(f"  {id1}-{id2}: {dist:.3f} m")

print_distances(anchors, "estimated anchors")
print_distances(real_anchors, "real anchors")

# Draw coordinate axes
axis_length = 80
ax.quiver(0, 0, 0, axis_length, 0, 0, color='r', arrow_length_ratio=0.1, label='X axis')
ax.quiver(0, 0, 0, 0, axis_length, 0, color='g', arrow_length_ratio=0.1, label='Y axis')
ax.quiver(0, 0, 0, 0, 0, axis_length, color='b', arrow_length_ratio=0.1, label='Z axis')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Estimated Anchor Positions with Coordinate Axes')
ax.legend()
ax.grid(True)
plt.tight_layout()
plt.show()
