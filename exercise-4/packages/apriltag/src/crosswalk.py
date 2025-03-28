#!/usr/bin/env python3

# potentially useful for part 2 of exercise 4

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
THROTTLE_RIGHT = 0.27
DIRECTION_RIGHT = 1

class CrossWalkNode(DTROS):

    def __init__(self, node_name):
        super(CrossWalkNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self._bridge = CvBridge()
        self._vehicle_name = os.environ['VEHICLE_NAME']

        with open("./packages/cv_undistort/src/csc22936.yaml", "r") as file:
            calib_data = yaml.safe_load(file)

        # Extract the camera matrix (K)
        self.K = np.array(calib_data["camera_matrix"]["data"]).reshape(3, 3)

        # Extract distortion coefficients (D)
        self.D = np.array(calib_data["distortion_coefficients"]["data"])
        
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
        self.yellow_lower = np.array([22, 120, 120], np.uint8)
        self.yellow_upper = np.array([28, 255, 255], np.uint8)
        self.blue_lower = np.array([100, 150, 50], np.uint8)  
        self.blue_upper = np.array([115, 255, 255], np.uint8)
        self.orange_lower = np.array([10, 120, 120], np.uint8)
        self.orange_upper = np.array([20, 255, 255], np.uint8)


        self._publisher_led = rospy.Publisher(signal_topic, LEDPattern, queue_size=1)

        self._publisher = rospy.Publisher('poster', Image, queue_size=1)
        self.rate = rospy.Rate(10)

        self.last_tag_id = None
        self.stop = False

        self.time_limit = 0.5
        self.start_time = None
        self.seen_line = 0

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
        else:
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher_wheel.publish(message)
            return False
        return True
    
    # def wait(self, initial_time, duration):
    #     if time.time() - initial_time < duration:
    #         message = WheelsCmdStamped(vel_left=0, vel_right=0)
    #         self._publisher.publish(message)
    #         # rate.sleep()
    #     else:
    #         return False
    #     return True
    
    def stop_wheels(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher_wheel.publish(stop)

    def detect_peDuckstrians(self, hsvFrame, undistorted_img):
        yellow_mask = cv2.inRange(hsvFrame, self.orange_lower, self.orange_upper)
        
        # yellow_mask = cv2.dilate(yellow_mask, np.ones((5, 5), "uint8"))
        contours, hierarchy = cv2.findContours(yellow_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)

        return len(contours) > 0


    def camera_callback(self, msg):
        if rospy.is_shutdown():
            self.stop_wheels()

        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        self.h, self.w = image.shape[:2]

        image = image[int(0.5 * self.h):, :]
        self.h, self.w = image.shape[:2]

        new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (self.w, self.h), alpha=1, centerPrincipalPoint=True)

        undistorted_img = cv2.undistort(image, self.K, self.D, None, new_K)
        hsvFrame = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2HSV)

        if self._ticks_right is None or self._ticks_left is None or self._starting_ticks_left is None:
            return

        if self.stop:
            print('STOPPING RIGHT NOW')
            if self.seen_line == 1:
                if time.time() - self.start_time < 1:
                    self.stop_wheels()
                else:
                    self.seen_line = 2
                # if time.time() - self.start_time < 3:
                #     self.stop_wheels()
            if self.seen_line == 2:
                if self.detect_peDuckstrians(hsvFrame, undistorted_img):
                    print("SAW YELLOW DUCKS")
                    self.stop_wheels()
                else:
                    self.stop = False

        else:
            starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self.drive_straight(starting, 2)

            # height, width, _ = imageFrame.shape

            
            blue_mask = cv2.inRange(hsvFrame, self.blue_lower, self.blue_upper)

            # kernel = np.ones((5, 5), "uint8")
            # blue_mask = cv2.dilate(blue_mask, np.ones((5, 5), "uint8"))

            # if self.detect_peDuckstrians(hsvFrame, undistorted_img):
            #     self.stop = True
            #     return
            # else:
            contours, hierarchy = cv2.findContours(blue_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
            if self.seen_line == 2:
                if len(contours) == 0:
                    self.seen_line = 0
                else:
                    return
            

            for point in contours:
                x, y = point[0][0]
                if y >= 0.45 * self.h:
                    print("GOT IN")
                    self.seen_line = 1
                    self.start_time = time.time()
                    self.stop = True
                    break


            # for pic, contour in enumerate(contours): 
            #     area = cv2.contourArea(contour) 
            #     if(area > 300): 
            #         x, y, w, h = cv2.boundingRect(contour) 
            #         undistorted_img = cv2.rectangle(undistorted_img, (x, y), 
            #                                 (x + w, y + h), 
            #                                 (0, 255, 0), 2) 
                    
            #         cv2.putText(undistorted_img, "Blue Colour", (x, y), 
            #                     cv2.FONT_HERSHEY_SIMPLEX, 
            #                     1.0, (255, 0, 0)) 
                    
            print(f"self.seen_line {self.seen_line}")

        message = self._bridge.cv2_to_imgmsg(undistorted_img, encoding="bgr8")

        # message = self._bridge.cv2_to_imgmsg(gray_image, encoding="passthrough")

        self._publisher.publish(message)

if __name__ == '__main__':
    # create the node
    node = CrossWalkNode(node_name='april_tag_detector')
    rospy.spin()
