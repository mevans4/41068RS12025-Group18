import numpy as np
from utils import detect_trees

def generate_circle_points(center, radius, num_points):
    angles = np.linspace(0, 2 * np.pi, num_points)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    return np.stack((x, y), axis=-1)

# Generate synthetic data for 2 trees
points1 = generate_circle_points((1, 1), 0.15, 30)
points2 = generate_circle_points((3, 2), 0.25, 40)
lidar_points = np.vstack((points1, points2))

# Run detection
trees = detect_trees(lidar_points)
print(trees)