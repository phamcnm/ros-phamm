#!/usr/bin/env python3

# potentially useful for part 1 of exercise 4

# import required libraries
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, LEDPattern

from dt_apriltags import Detector
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String, ColorRGBA
import time
import yaml

THROTTLE_LEFT = 0.23
DIRECTION_LEFT = 1
THROTTLE_RIGHT = 0.268
DIRECTION_RIGHT = 1

class ApriltagNode(DTROS):

    def __init__(self, node_name):
        super(ApriltagNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self._bridge = CvBridge()
        self._vehicle_name = os.environ['VEHICLE_NAME']

        with open("./packages/cv_undistort/src/csc22936.yaml", "r") as file:
            calib_data = yaml.safe_load(file)

        # Extract the camera matrix (K)
        self.K = np.array(calib_data["camera_matrix"]["data"]).reshape(3, 3)

        # Extract distortion coefficients (D)
        self.D = np.array(calib_data["distortion_coefficients"]["data"])

        # initialize dt_apriltag detector
        self.at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=2.0,
                       quad_sigma=0.0,
                       refine_edges=0,
                       decode_sharpening=0.25,
                       debug=0)
        
        # subscribe to camera feed
        camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        signal_topic = f"/{self._vehicle_name}/led_emitter_node/led_pattern"

        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"

        self._ticks_left = None
        self._ticks_right = None
        self._starting_ticks_left = None
        self._starting_ticks_right = None

        self._publisher_wheel = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT 
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        self._radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 100)

        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)


        self.sub = rospy.Subscriber(camera_topic, CompressedImage, self.camera_callback)

        self.red_lower = np.array([136, 87, 111], np.uint8)
        self.red_upper = np.array([180, 255, 255], np.uint8)

        self._publisher_led = rospy.Publisher(signal_topic, LEDPattern, queue_size=1)

        self._publisher_camera = rospy.Publisher('poster', Image, queue_size=1)
        self.rate = rospy.Rate(10)

        self.last_tag_id = None
        self.stop = False

        self.time_limit = 0.5
        self.start_time = None
        # self.seen_something = False
        self.has_stopped = False
        self.seen_tag = False
        self.rate = rospy.Rate(100)

    def callback_left(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        # store data value
        self._ticks_left = data.data
        if not self._starting_ticks_left:
            self._starting_ticks_left = self._ticks_left

    def callback_right(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        # store data value
        self._ticks_right = data.data
        if not self._starting_ticks_right:
            self._starting_ticks_right = self._ticks_right


    def sign_to_led(self, **kwargs):
        pass

    def process_image(self, **kwargs):
        pass

    def publish_augmented_img(self, **kwargs):
        pass

    def publish_leds(self, colors):
        rgba_val = ColorRGBA(r=colors[0], g=colors[1], b=colors[2], a=1)
        self._publisher_led.publish(LEDPattern(rgb_vals=[rgba_val,rgba_val,rgba_val,rgba_val,rgba_val]))
        # self.rate.sleep()

    def detect_tag(self, **kwargs):
        pass

    def drive_straight(self, starting, distance):
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
        if (cur_pos - starting) / 135 < distance:
            cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self._publisher_wheel.publish(message)
            # rate.sleep()
            return False
        else:
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher_wheel.publish(message)
            return True
    
    # def wait(self, initial_time, duration):
    #     if time.time() - initial_time < duration:
    #         message = WheelsCmdStamped(vel_left=0, vel_right=0)
    #         self._publisher.publish(message)
    #         # rate.sleep()
    #     else:
    #         return False
    #     return True

    def react_to_detection(self, tag_id):
        if tag_id == 21 or tag_id == 22:
            self.publish_leds([1,0,0]) # stop
        elif tag_id == 50 or tag_id == 133:
            self.publish_leds([0,0,1]) # T
        else:
            self.publish_leds([0,1,0]) # UA

    def react_to_stopping(self):
        return
    
    def stop_wheels(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher_wheel.publish(stop)

    def define_state(self, last_tag_id):
        if last_tag_id == 21 or last_tag_id == 22:
            self.time_limit = 3 # stop
        elif last_tag_id == 50 or last_tag_id == 133:
            self.time_limit = 2 # T
        else:
            self.time_limit = 1 # UA

    def camera_callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        self.h, self.w = image.shape[:2]

        new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (self.w, self.h), alpha=1, centerPrincipalPoint=True)

        undistorted_img = cv2.undistort(image, self.K, self.D, None, new_K)
        if not self.seen_tag:
            cut_image = undistorted_img[:, int(0.5 * self.w):]
            # self.h, self.w = cut_image.shape[:2]

            gray_image = cv2.cvtColor(cut_image, cv2.COLOR_BGR2GRAY)

            tags = self.at_detector.detect(gray_image, estimate_tag_pose=False, camera_params=None, tag_size=None)

            if len(tags) > 0:
                # self.seen_something = True
                self.react_to_detection(tags[0].tag_id)
                self.last_tag_id = tags[0].tag_id
                self.seen_tag = True
                print(f"saw tag {tags[0].tag_id}")
        
        # loop over the AprilTag detection results
        # for r in tags:
        #     # extract the bounding box (x, y)-coordinates for the AprilTag
        #     # and convert each of the (x, y)-coordinate pairs to integers
        #     (ptA, ptB, ptC, ptD) = r.corners
        #     ptB = (int(ptB[0]), int(ptB[1]))
        #     ptC = (int(ptC[0]), int(ptC[1]))
        #     ptD = (int(ptD[0]), int(ptD[1]))
        #     ptA = (int(ptA[0]), int(ptA[1]))
        #     # draw the bounding box of the AprilTag detection
        #     cv2.line(gray_image, ptA, ptB, (0, 255, 0), 2)
        #     cv2.line(gray_image, ptB, ptC, (0, 255, 0), 2)
        #     cv2.line(gray_image, ptC, ptD, (0, 255, 0), 2)
        #     cv2.line(gray_image, ptD, ptA, (0, 255, 0), 2)
        #     # draw the center (x, y)-coordinates of the AprilTag
        #     (cX, cY) = (int(r.center[0]), int(r.center[1]))
        #     cv2.circle(gray_image, (cX, cY), 5, (0, 0, 255), -1)
        #     # draw the tag family on the image
            
        #     tagId = str(r.tag_id)
        #     print(tagId)
        #     if tagId is not None:
        #         cv2.putText(gray_image, tagId, (int(r.center[0]), int(r.center[1])),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if self._ticks_right is None or self._ticks_left is None or self._starting_ticks_left is None:
            return

        if self.stop and not self.has_stopped:
            print(f"time limit {self.time_limit}")
            if time.time() - self.start_time < self.time_limit:
                print('STOPPING RIGHT NOW')
                self.stop_wheels()
                return
            else:
                self.publish_leds([1,1,1])
                self.stop = False
                self.has_stopped = True
                self.starting_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)

        if not self.stop:
            starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self.drive_straight(starting, 2)

            # height, width, _ = imageFrame.shape
            hsvFrame = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2HSV) 
            red_mask = cv2.inRange(hsvFrame, self.red_lower, self.red_upper)

            # kernel = np.ones((5, 5), "uint8")
            # red_mask = cv2.dilate(red_mask, kernel)5
            # res_red = cv2.bitwise_and(imageFrame, imageFrame, mask = red_mask)
            contours, hierarchy = cv2.findContours(red_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
            for point in contours:
                x, y = point[0][0]
                if y >= 0.7 * self.h:
                    self.stop = True
                    self.start_time = time.time()
                    print('GOT IN HEREEEEE: STOPPING NOW')
                    break

                    # self.image = undistorted_img

            # for pic, contour in enumerate(contours): 
            #     area = cv2.contourArea(contour) 
            #     if(area > 300): 
            #         x, y, w, h = cv2.boundingRect(contour) 
            #         undistorted_img = cv2.rectangle(undistorted_img, (x, y), 
            #                                 (x + w, y + h), 
            #                                 (0, 255, 0), 2) 
                    
            #         cv2.putText(undistorted_img, "Red Colour", (x, y), 
            #                     cv2.FONT_HERSHEY_SIMPLEX, 
            #                     1.0, (0, 255, 0)) 

            self.define_state(self.last_tag_id)


        # message = self._bridge.cv2_to_imgmsg(undistorted_img, encoding="bgr8")
        # message = self._bridge.cv2_to_imgmsg(gray_image, encoding="passthrough")

        # self._publisher_camera.publish(message)
        # self.rate.sleep()



if __name__ == '__main__':
    # create the node
    node = ApriltagNode(node_name='apriltag_detector_node')
    rospy.spin()
    
