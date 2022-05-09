import laspy
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull

class ClusteringModel:
    def __init__(self, input_las_path: str):
        with laspy.open(input_las_path) as f:
            las = f.read()
        self.points_xyz = np.column_stack([las.x, las.y, las.z])
        self.point_heights = las.HeightAboveGround
        self.polygon_type = "RECTANGLE"
        self.cluster_threshold = 1000
        self.stride = 100

    # Selects points in the point cloud that fill a bounding range
    def select_points(self, points_xy, min_x, max_x, min_y, max_y):
        select_indices = np.where((min_x < points_xy[:,0]) & (points_xy[:,0] < max_x) & (min_y < points_xy[:,1]) & (points_xy[:,1] < max_y))
        return points_xy[select_indices], select_indices

    def create_bounding_box(self, min_bound, cluster_points):
        z_min = min(cluster_points[:,2])
        z_max = max(cluster_points[:,2])

        bottom_face = np.column_stack((min_bound, [z_min] * min_bound.shape[0]))
        top_face = np.column_stack((min_bound, [z_max] * min_bound.shape[0]))
        bbox = np.vstack((bottom_face, top_face))

        return bbox

    def convex_hull(self, points):
        convex_hull = points[ConvexHull(points).vertices]
        return convex_hull

    def minimum_bounding_rectangle(self, points):
        """
        Find the smallest bounding rectangle for a set of points.
        Returns a set of points representing the corners of the bounding box.

        :param points: an nx2 matrix of coordinates
        :rval: an nx2 matrix of coordinates
        """
        pi2 = np.pi/2.

        # get the convex hull for the points
        hull_points = points[ConvexHull(points).vertices]

        # calculate edge angles
        edges = np.zeros((len(hull_points)-1, 2))
        edges = hull_points[1:] - hull_points[:-1]

        angles = np.zeros((len(edges)))
        angles = np.arctan2(edges[:, 1], edges[:, 0])

        angles = np.abs(np.mod(angles, pi2))
        angles = np.unique(angles)

        rotations = np.vstack([
            np.cos(angles),
            np.cos(angles-pi2),
            np.cos(angles+pi2),
            np.cos(angles)]).T
        rotations = rotations.reshape((-1, 2, 2))

        # apply rotations to the hull
        rot_points = np.dot(rotations, hull_points.T)

        # find the bounding points
        min_x = np.nanmin(rot_points[:, 0], axis=1)
        max_x = np.nanmax(rot_points[:, 0], axis=1)
        min_y = np.nanmin(rot_points[:, 1], axis=1)
        max_y = np.nanmax(rot_points[:, 1], axis=1)

        # find the box with the best area
        areas = (max_x - min_x) * (max_y - min_y)
        best_idx = np.argmin(areas)

        # return the best box
        x1 = max_x[best_idx]
        x2 = min_x[best_idx]
        y1 = max_y[best_idx]
        y2 = min_y[best_idx]
        r = rotations[best_idx]

        rval = np.zeros((4, 2))
        rval[0] = np.dot([x1, y2], r)
        rval[1] = np.dot([x2, y2], r)
        rval[2] = np.dot([x2, y1], r)
        rval[3] = np.dot([x1, y1], r)

        return rval


    def run_clustering(self, min_samples: int, eps: float):
        """
        Runs the DBSCAN algorithm.

        Takes a point cloud and produces a list of bounding boxes that represent obstacles for our drones to avoid.

        Parameters:
            min_samples(int): Minimum samples for the clustering algorithm
            eps(float): Epsilon value for the clustering algorithm. This changes the distance between points required for them to be considered to be in the same cluster.

        Output: ndarray of shape (n_points, n_cluster, 8, 3)
        """
        self.bbox_vertices = []
        self.bbox_hags = []
        clustering = DBSCAN(eps = 10, min_samples = 150).fit(self.points_xyz)

        cluster_labels = np.array(clustering.labels_)
        cluster_labels_unique = np.unique(clustering.labels_)
        cluster_labels_unique = cluster_labels_unique[cluster_labels_unique != 1]

        for cluster_label in cluster_labels_unique:
            cluster_indices = np.where(cluster_label == cluster_labels)
            points_in_cluster = self.points_xyz[cluster_indices]
            min_x = int(np.min(points_in_cluster[:,0], axis = 0))
            max_x = round(np.max(points_in_cluster[:,0], axis = 0))
            min_y = int(np.min(points_in_cluster[:,1], axis = 0))
            max_y = round(np.max(points_in_cluster[:,1], axis = 0))
            cluster_bounding_area = max((max_x - min_x), (max_y - min_y))

            if cluster_bounding_area > self.cluster_threshold:
                for x in range(min_x, max_x, self.stride):
                    for y in range(min_y, max_x, self.stride):
                        points_in_box, points_in_box_indices = self.select_points(points_in_cluster, x, x + self.stride, y, y + self.stride)
                        if points_in_box.shape[0] >= 3:
                            points_in_box_xy = np.column_stack((points_in_box[:,0], points_in_box[:,1]))
                            if points_in_box.shape[0] > 3:
                                min_bounding_poly = self.minimum_bounding_rectangle(points_in_box_xy)
                            self.bbox_vertices.append(self.create_bounding_box(min_bounding_poly, points_in_box))
                            cluster_heights = self.point_heights[cluster_indices]
                            max_height = np.amax(cluster_heights[points_in_box_indices])
                            self.bbox_hags.append(max_height)

            else:
                points_xy = np.column_stack((points_in_cluster[:,0], points_in_cluster[:,1]))
                min_bounding_poly = self.minimum_bounding_rectangle(points_xy)
                self.bbox_vertices.append(self.create_bounding_box(min_bounding_poly, points_in_cluster))
                max_height = np.amax(self.point_heights[cluster_indices])
                self.bbox_hags.append(max_height)
        return self.bbox_vertices

