import laspy
from sklearn.cluster import DBSCAN
import numpy as np

point_cloud_file = r"../data/test2.las"
with laspy.open(point_cloud_file) as f:
    las = f.read()
points_xyz = np.column_stack([las.x, las.y, las.z])
clustering = DBSCAN(eps = 10, min_samples = 1000).fit(points_xyz)