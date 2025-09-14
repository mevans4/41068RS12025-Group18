import numpy as np

class TreeDetector:
    def detect_trees(self, lidar_data):
        """
        Detect trees from LiDAR data.
        lidar_data: list or np.array of (x, y) or (x, y, z) points
        Returns: list of detected trees (each as a cluster of points)
        """
        # Example: simple clustering based on distance threshold
        trees = []
        visited = set()
        threshold = 0.5  # meters, adjust as needed

        for i, point in enumerate(lidar_data):
            if i in visited:
                continue
            cluster = [point]
            visited.add(i)
            for j, other_point in enumerate(lidar_data):
                if j in visited:
                    continue
                if np.linalg.norm(np.array(point) - np.array(other_point)) < threshold:
                    cluster.append(other_point)
                    visited.add(j)
            if len(cluster) > 5:  # minimum points for a tree
                trees.append(cluster)
        return trees

    def calculate_tree_diameter(self, tree_data):
        """
        Calculate the diameter of a tree from its cluster of points.
        tree_data: list of (x, y) or (x, y, z) points
        Returns: diameter in meters
        """
        points = np.array(tree_data)
        if points.shape[0] < 2:
            return 0.0
        # Diameter: max distance between any two points in the cluster
        dists = np.linalg.norm(points - points[:, None], axis=-1)
        diameter = np.max(dists)
        return diameter