import numpy as np
from xml.dom import minidom
from cluster import ClusteringModel
import sys

class ClusteringTool():
    def __init__(self, las_file, db_file):
        self.max_eps = 10
        self.min_points = 1000
        self.point_cloud_file = las_file
        self.output_path = db_file
        self.output_geometry = "RECTANGLE"
        self.sliding_window_min_length = 1000
        self.sliding_window_stride = 100
        self.clustering_algorithm = "DBSCAN"

        self.model = ClusteringModel(self.point_cloud_file)

    def run_clustering(self):
        self.model.run_clustering(self.min_points, self.max_eps)

    def add_obstacles_to_db(self):
        self.obstacle_db_root = minidom.Document()
        feature_items = []
        row_count = 0

        for i in range(len(self.model.bbox_vertices)):
            row_count += 1

            bbox = self.model.bbox_vertices[i]
            half_index = int((len(bbox) / 2))
            bbox_square = bbox[0:half_index, 0:2]
            hag = self.model.bbox_hags[i]
            max_z = bbox[half_index,2]

            feature_item = self.obstacle_db_root.createElement("FEATUREITEM")
            feature_item.setAttribute("FeatureType", "Region")
            feature_items.append(feature_item)

            string_id = self.obstacle_db_root.createElement("STRING_ID")
            string_id.setAttribute("StringID", str(row_count))
            feature_item.appendChild(string_id)

            elevation = self.obstacle_db_root.createElement("ELEVATION")
            elevation.setAttribute("Elev_m", str(max_z))
            feature_item.appendChild(elevation)

            num_points = self.obstacle_db_root.createElement("NUMPOINTS")
            num_points.setAttribute("NumPoints", "4")
            feature_item.appendChild(num_points)

            for points in bbox_square:
                point_2d = self.obstacle_db_root.createElement("Point2D")
                point_2d.setAttribute("LonDeg", str(points[0]))
                point_2d.setAttribute("LatDeg", str(points[1]))
                feature_item.appendChild(point_2d)

        features_format = self.obstacle_db_root.createElement("FEATURES_FORMAT_201707")
        self.obstacle_db_root.appendChild(features_format)

        header_info = self.obstacle_db_root.createElement("HEADERINFO")
        header_info.setAttribute("NumFeatures", str(row_count))
        header_info.setAttribute("DefaultValue", str(0))
        header_info.setAttribute("ReplaceMethod", "ALWAYS")
        features_format.appendChild(header_info)

        for feature_item in feature_items:
            features_format.appendChild(feature_item)

    def write_xml(self):
        file_handle = open(self.output_path, "w")
        pretty_xml = self.obstacle_db_root.toprettyxml()
        file_handle.write(pretty_xml)
        file_handle.close()

if __name__ == '__main__':
    las_file = sys.argv[1]
    db_file = sys.argv[2]
    obstacle_detector = ClusteringTool(las_file, db_file)
    obstacle_detector.run_clustering()
    obstacle_detector.add_obstacles_to_db()
    obstacle_detector.write_xml()