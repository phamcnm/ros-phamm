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

THROTTLE_LEFT = 0.32
DIRECTION_LEFT = 1
THROTTLE_RIGHT = 0.355
DIRECTION_RIGHT = 1

class PController:
    def __init__(self, Kp):
        self.Kp = Kp
        self.previous_error = 0
        self.delta = 5

    def compute(self, error, dt):
        error = min(error, self.previous_error + self.delta)
        error = max(error, self.previous_error - self.delta)
        output = self.Kp * error
        self.previous_error = error
        return output

class SafeNavigationNode(DTROS):

    def __init__(self, node_name):
        super(SafeNavigationNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

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

        self.start_location = None

        self.sub = rospy.Subscriber(camera_topic, CompressedImage, self.camera_callback)

        self._publisher_led = rospy.Publisher(signal_topic, LEDPattern, queue_size=1)

        self._publisher_camera = rospy.Publisher('poster', Image, queue_size=1)
        self.isFound = False
        self.centers = None

        self.white_lower = np.array([0, 0, 170], np.uint8)
        self.white_upper = np.array([180, 25, 255], np.uint8)
        self.yellow_lower = np.array([22, 120, 120], np.uint8)
        self.yellow_upper = np.array([28, 255, 255], np.uint8)

        self.phase = 0

        self.p = PController(Kp=0.5)
        self.prev_steering = 0
        self.delta = 0.3

        self.rate = rospy.Rate(20)

        self.publish_leds([0, 0, 0])

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

    def publish_leds(self, colors):
        rgba_val = ColorRGBA(r=colors[0], g=colors[1], b=colors[2], a=1)
        self._publisher_led.publish(LEDPattern(rgb_vals=[rgba_val,rgba_val,rgba_val,rgba_val,rgba_val]))

    def sign_to_led(self, **kwargs):
        pass

    def process_image(self, **kwargs):
        pass

    def publish_augmented_img(self, **kwargs):
        pass

    def detect_tag(self, **kwargs):
        pass

    def get_lane_error(self, white_contours, yellow_contours, side):
        max_white_point = (0, 0)
        max_yellow_point = (self.w, 0)

        # should have passed the corner test, there should be items in bottom screen (ie, >= 0.9h)

        for point in white_contours:
            x, y = point[0][0]
            # if y < 0.6 * self.h: # ignore anything far ahead
            #     continue
            if y >= max_white_point[1]:
                max_white_point = (x, y)

        for point in yellow_contours:
            x, y = point[0][0]
            # if y < 0.6 * self.h: # ignore anything far ahead
            #     continue
            if y >= max_yellow_point[1]:
                max_yellow_point = (x, y)

        
        avg = (max_white_point[0] + max_yellow_point[0]) / 2
        cur = self.w / 2

        # if side == 0: # left, then yellow[x] > white[x]
        #     prop = (cur - avg) / (max_yellow_point[0] - max_white_point[0])
        # elif side == 1: # right, then white[x] > yellow[x]
        #     prop = (cur - avg) / (max_white_point[0] - max_yellow_point[0])
        prop = (cur - avg) / self.w
        # prop = (cur - avg) / (max_white_point[0] - max_yellow_point[0])
        # print('in get error, prop', prop)
        return prop

    def drive_straight(self, starting, distance, mult_left=1, mult_right=1):
        message = WheelsCmdStamped(vel_left=mult_left * self._vel_left, vel_right=mult_right * self._vel_right)
        cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
        if (cur_pos - starting) / 135 < distance:
            cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self._publisher_wheel.publish(message)
            self.rate.sleep()
            return False
        else:
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher_wheel.publish(message)
            return True
        
    
    def rotate(self, starting, degree, direction='right'):
        cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
        if direction == 'right':
            message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=-self._vel_right)
            if (cur_pos-starting) / 135 < degree:
                cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
                self._publisher_wheel.publish(message)
                self.rate.sleep()
                return False # not yet done
        else:
            message = WheelsCmdStamped(vel_left=-self._vel_left, vel_right=self._vel_right)
            if (cur_pos - starting) / 135 > -degree:
                cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
                self._publisher_wheel.publish(message)
                self.rate.sleep()
                return False # not yet done
        return True # done rotating
    
    def set_steering(self, steering):
        mult = 0.32
        print('In set steering: %f' % steering)
        message = WheelsCmdStamped(vel_left=(1-mult*steering)*self._vel_left, vel_right= (1+mult*steering)*self._vel_right)
        self._publisher_wheel.publish(message)
        # self.rate_wheel.sleep()
    
    def stop_wheels(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher_wheel.publish(stop)

    def next_phase(self):
        self.phase += 1
        self.start_time = time.time()
        self.start_location = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)

    def camera_callback(self, msg):
        if rospy.is_shutdown():
            self.stop_wheels()

        undistorted_img = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        # self.h, self.w = image.shape[:2]

        # # image = image[int(0.5 * self.h):, :]
        # # self.h, self.w = image.shape[:2]

        # new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (self.w, self.h), alpha=1, centerPrincipalPoint=True)

        # undistorted_img = cv2.undistort(image, self.K, self.D, None, new_K)

        # message = self._bridge.cv2_to_imgmsg(undistorted_img, encoding="bgr8")

        # self._publisher_camera.publish(message)
        print(self.phase)

        if self._ticks_right is None or self._ticks_left is None or self._starting_ticks_left is None:
            return
        
        if self.phase == 0:
            starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self.drive_straight(starting, 0.5, mult_left=0.56, mult_right=0.6)
            # gray = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)
            # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            # shape = (3, 7)
            [is_found, centers] = cv2.findCirclesGrid(undistorted_img, (7, 3), flags=cv2.CALIB_CB_SYMMETRIC_GRID)
            if is_found or centers is not None:
                self.stop_wheels()
                print('FOUND')
                self.next_phase()

        elif self.phase == 1:
            if time.time() - self.start_time < 3:
                self.stop_wheels()
            elif self.rotate(starting=self.start_location, degree=0.03, direction='right'):
                self.next_phase()
            
        elif self.phase == 2:
            if time.time() - self.start_time < 1:
                self.stop_wheels()
            elif self.drive_straight(self.start_location, 0.25):
                self.next_phase()

        elif self.phase == 3:
            if time.time() - self.start_time < 1:
                self.stop_wheels()
            elif self.rotate(starting=self.start_location, degree=0.028, direction='left'):
                self.next_phase()
        
        elif self.phase == 4:
            if time.time() - self.start_time < 1:
                self.stop_wheels()
            elif self.drive_straight(self.start_location, 0.25):
                self.next_phase()
            # if time.time() - self.start_time < 1:
            #     self.stop_wheels()
            #     return
            # curr_location = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            # if curr_location - self.start_location > 0.5:
            #     self.next_phase()

            # hsvFrame = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2HSV)
            # white_mask = cv2.inRange(hsvFrame, self.white_lower, self.white_upper)
            # yellow_mask = cv2.inRange(hsvFrame, self.yellow_lower, self.yellow_upper)

            # white_contours, white_hierarchy = cv2.findContours(white_mask, 
            #                                 cv2.RETR_TREE, 
            #                                 cv2.CHAIN_APPROX_SIMPLE)

            # yellow_contours, yellow_hierarchy = cv2.findContours(yellow_mask, 
            #                                 cv2.RETR_TREE, 
            #                                 cv2.CHAIN_APPROX_SIMPLE)
            
            # dt = 0.01  # Time step
            # error = self.get_lane_error(white_contours, yellow_contours, side=0)
            # steering_angle = self.p.compute(error, dt)
            
            # steering_angle = min(steering_angle, self.prev_steering + self.delta)
            # steering_angle = max(steering_angle, self.prev_steering - self.delta)
            # self.prev_steering = steering_angle
            # self.set_steering(steering_angle)

        elif self.phase == 5:
            if time.time() - self.start_time < 1:
                self.stop_wheels()
            elif self.rotate(starting=self.start_location, degree=0.025, direction='left'):
                self.next_phase()

        elif self.phase == 6:
            if time.time() - self.start_time < 1:
                self.stop_wheels()
            elif self.drive_straight(self.start_location, 0.25):
                self.next_phase()

        elif self.phase == 7:
            if time.time() - self.start_time < 1:
                self.stop_wheels()
            elif self.rotate(starting=self.start_location, degree=0.025, direction='right'):
                self.next_phase()

        elif self.phase == 8:
            if time.time() - self.start_time < 1:
                self.stop_wheels()
            elif self.drive_straight(self.start_location, 0.3):
                self.next_phase()
            # if time.time() - self.start_time < 1:
            #     self.stop_wheels()
            #     return
            # curr_location = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            # if curr_location - self.start_location > 0.4:
            #     self.next_phase()

            # hsvFrame = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2HSV)
            # white_mask = cv2.inRange(hsvFrame, self.white_lower, self.white_upper)
            # yellow_mask = cv2.inRange(hsvFrame, self.yellow_lower, self.yellow_upper)

            # white_contours, white_hierarchy = cv2.findContours(white_mask, 
            #                                 cv2.RETR_TREE, 
            #                                 cv2.CHAIN_APPROX_SIMPLE)

            # yellow_contours, yellow_hierarchy = cv2.findContours(yellow_mask, 
            #                                 cv2.RETR_TREE, 
            #                                 cv2.CHAIN_APPROX_SIMPLE)
            
            # dt = 0.01  # Time step
            # error = self.get_lane_error(white_contours, yellow_contours, side=1)
            # steering_angle = self.p.compute(error, dt)
            
            # steering_angle = min(steering_angle, self.prev_steering + self.delta)
            # steering_angle = max(steering_angle, self.prev_steering - self.delta)
            # self.prev_steering = steering_angle
            # self.set_steering(steering_angle)

        elif self.phase == 9:
            self.stop_wheels()


if __name__ == '__main__':
    # create the node
    node = SafeNavigationNode(node_name='safe_navigator')
    rospy.spin()
