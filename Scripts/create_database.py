import numpy as np
from xml.dom import minidom
from cluster import ClusteringModel
import sys
import socket
import tqdm
import os
import subprocess
import json

class ClusteringTool():
    def __init__(self, las_file, db_file):
        self.max_eps = 10
        self.min_points = 1000
        self.point_cloud_file = las_file
        self.filtered_point_cloud_file = "filtered_test.las"
        self.output_path = db_file
        self.output_geometry = "RECTANGLE"
        self.sliding_window_min_length = 1000
        self.sliding_window_stride = 100
        self.clustering_algorithm = "DBSCAN"

    def run_clustering(self):
        self.model = ClusteringModel(self.filtered_point_cloud_file)
        print("model defined...")
        self.model.run_clustering(self.min_points, self.max_eps)
        print("run clustering done...")

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

    def upload_to_database(self):
        SEPARATOR = "<SEPARATOR>"
        BUFFER_SIZE = 4096 # send 4096 bytes each time step
        host = "172.88.24.161" # the ip address or hostname of the server, the receiver
        port = 5001 # the port, let's use 5001
        filename = self.output_path # the name of file we want to send, make sure it exists
        filesize = os.path.getsize(filename) # get the file size

        s = socket.socket() # create the client socket

        print(f"[+] Connecting to {host}:{port}")
        s.connect((host, port))
        print("[+] Connected.")

        # send the filename and filesize
        s.send(f"{filename}{SEPARATOR}{filesize}".encode())

        # start sending the file
        progress = tqdm.tqdm(range(filesize), f"Sending {filename}", unit="B", unit_scale=True, unit_divisor=1024)
        with open(filename, "rb") as f:
            while True:
                # read the bytes from the file
                bytes_read = f.read(BUFFER_SIZE)
                if not bytes_read:
                    # file transmitting is done
                    break
                # we use sendall to assure transmission in
                # busy networks
                s.sendall(bytes_read)
                # update the progress bar
                progress.update(len(bytes_read))
        # close the socket

    def filter(self):
        data = [
            {
                "type": "readers.las",
                "filename": self.point_cloud_file
            },
            {
                "type": "filters.smrf"
            },
            {
                "type": "filters.hag_nn"
            },
            {
                "type": "filters.range",
                "limits": "HeightAboveGround[10:]"
            },
            {
                "type": "filters.range",
                "limits": "Classification[1:1]",
                "tag": "Classify"
            },
            {
                "type": "writers.las",
                "filename": "filtered_test.las",
                "extra_dims": "HeightAboveGround=double"
            }
            ]
        with open("pdal_filter.json", "w") as outfile:
            json.dump(data, outfile)

        subprocess.run(["pdal", "pipeline", "pdal_filter.json"])
        print("filtered...")

if __name__ == '__main__':
    las_file = sys.argv[1]
    db_file = sys.argv[2]
    obstacle_detector = ClusteringTool(las_file, db_file)
    obstacle_detector.filter()
    print("starting clustering...")
    obstacle_detector.run_clustering()
    obstacle_detector.add_obstacles_to_db()
    obstacle_detector.write_xml()
    #obstacle_detector.upload_to_database()