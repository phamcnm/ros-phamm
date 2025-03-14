#!/usr/bin/env python3

import os
import rospy
import cv2
import yaml  # To read YAML files
import numpy as np

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class Undistort(DTROS):
    def __init__(self, node_name):
        super(Undistort, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here
        print("Current Directory:", os.getcwd())
        self._vehicle_name = os.environ['VEHICLE_NAME']
        camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._subsriber = rospy.Subscriber(camera_topic, CompressedImage, self.callback)
        self._publisher = rospy.Publisher(f"/undistorted", Image, queue_size=10)
        self._bridge = CvBridge()

        # Load the YAML file
        with open("./packages/cv_undistort/src/csc22936.yaml", "r") as file:
            calib_data = yaml.safe_load(file)

        # Extract the camera matrix (K)
        self.K = np.array(calib_data["camera_matrix"]["data"]).reshape(3, 3)

        # Extract distortion coefficients (D)
        self.D = np.array(calib_data["distortion_coefficients"]["data"])

        # Extract image dimensions
        self.image_width = calib_data["image_width"]
        self.image_height = calib_data["image_height"]
        
    def callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        h, w = image.shape[:2]

        new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), alpha=1, centerPrincipalPoint=True)

        undistorted_img = cv2.undistort(image, self.K, self.D, None, new_K)
        
        undistorted_msg = self._bridge.cv2_to_imgmsg(undistorted_img, encoding="rgb8")

        self._publisher.publish(undistorted_msg)

if __name__ == '__main__':
    node = Undistort(node_name='navigation_control_node')
    rospy.spin()