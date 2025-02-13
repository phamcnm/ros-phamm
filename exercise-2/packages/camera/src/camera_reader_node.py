#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image

import cv2
from cv_bridge import CvBridge


class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self._publisher = rospy.Publisher('poster', Image, queue_size=10)
        self.image = None


    def callback(self, msg):
        # convert JPEG bytes to CV image
        self.image = self._bridge.compressed_imgmsg_to_cv2(msg)
        print(self.image.shape)
        # display frame
        # cv2.imshow(self._window, image)
        # cv2.waitKey(1)

        grayscale = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Grayscale', grayscale)
        # cv2.waitKey(1)

        imageText = grayscale.copy()
        #let's write the text you want to put on the image
        text = 'Duck %s says, \'Cheese! Capturing %s - quack-tastic!\'' % (self._vehicle_name, str(self.image.shape[0]) + 'X' + str(self.image.shape[1]))
        #org: Where you want to put the text
        org = (50,350)
        # write the text on the input image
        cv2.putText(imageText, text, org, fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (250,225,100))
        # display the output image with text over it
        cv2.imshow("Section I.3 Part c",imageText)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        rate = rospy.Rate(1)
        message = self._bridge.cv2_to_imgmsg(imageText, encoding="passthrough")
        print(type(message))
        while not rospy.is_shutdown():
            self._publisher.publish(message)
            rate.sleep()


    # def run(self):
    #     # publish message every 1 second (1 Hz)


if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # node.run()

    # keep spinning
    rospy.spin()
