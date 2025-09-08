import numpy as np
from sklearn.cluster import DBSCAN

def detect_trees(lidar_points, eps=0.2, min_samples=5):
    """
    Detect tree trunks from LiDAR points.
    Args:
        lidar_points: np.ndarray of shape (N, 2), each row is (x, y)
        eps: DBSCAN clustering parameter (distance threshold)
        min_samples: DBSCAN clustering parameter (minimum points per cluster)
    Returns:
        List of dicts: [{'center': (x, y), 'width': diameter}, ...]
    """
    # Cluster points
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(lidar_points)
    labels = clustering.labels_
    trees = []
    for label in set(labels):
        if label == -1:
            continue  # noise
        cluster = lidar_points[labels == label]
        center = cluster.mean(axis=0)
        # Estimate diameter as max distance from center * 2
        distances = np.linalg.norm(cluster - center, axis=1)
        diameter = distances.max() * 2
        trees.append({'center': tuple(center), 'width': diameter})
    return trees