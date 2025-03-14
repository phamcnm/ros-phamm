#!/usr/bin/env python3

# potentially useful for question - 1.6

# import required libraries
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, LEDPattern
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String, ColorRGBA
import time

BOTTOM = 10

# throttle and direction for each wheel
THROTTLE_LEFT = 0.23
DIRECTION_LEFT = 1
THROTTLE_RIGHT = 0.25
DIRECTION_RIGHT = 1

class BehaviorController(DTROS):
    def __init__(self, node_name):
        super(BehaviorController, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here
        self._bridge = CvBridge()
        self._vehicle_name = os.environ['VEHICLE_NAME']
        # construct subscriber
        camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub = rospy.Subscriber(camera_topic, CompressedImage, self.callback)
        # self._publisher_image = rospy.Publisher('detection', Image, queue_size=1)
        self.image = None
        # call navigation control node
        
        # define parametersfrom std_msgs.msg import String, ColorRGBA
        
        # Color ranges in HSVpass
        self.red_lower = np.array([136, 87, 111], np.uint8)
        self.red_upper = np.array([180, 255, 255], np.uint8)
        self.green_lower = np.array([35, 100, 100], np.uint8)  
        self.green_upper = np.array([85, 255, 255], np.uint8)  
        self.blue_lower = np.array([100, 150, 50], np.uint8)  
        self.blue_upper = np.array([115, 255, 255], np.uint8)

        self.current_red = None
        self.current_blue = None
        self.current_green = None

        
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        signal_topic = f"/{self._vehicle_name}/led_emitter_node/led_pattern"

        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"

        self._ticks_left = None
        self._ticks_right = None
        self._starting_ticks_left = None
        self._starting_ticks_right = None

        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT 
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        self._radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 100)

        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

        # for LED
        self._publisher_led = rospy.Publisher(signal_topic, LEDPattern, queue_size=1)

        self.stop = False
        self.start_time = time.time()
        self.state = 'regular'
        self.starting_pos = None

        self.rate = rospy.Rate(100)

        empty = ColorRGBA(r=0, g=0, b=0, a=1)
        self._publisher_led.publish(LEDPattern(rgb_vals=[empty,empty,empty,empty,empty]))

        self.stop_wheels()
        self.execute_initial_led_behavior()
        

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

        
    def set_led_pattern(self, colors):
        rgba_val = ColorRGBA(r=colors[0], g=colors[1], b=colors[2], a=1)
        self._publisher_led.publish(LEDPattern(rgb_vals=[rgba_val,rgba_val,rgba_val,rgba_val,rgba_val]))
        self.rate.sleep()

    def drive_straight(self, rate, starting, distance):
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
        if (cur_pos - starting) / 135 < distance:
            cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self._publisher.publish(message)
            rate.sleep()
        else:
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(message)
            return False
        return True

    def rotate(self, rate, starting, degree, direction='right'):
        cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
        if direction == 'right':
            message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=-self._vel_right)
        else:
            message = WheelsCmdStamped(vel_left=-self._vel_left, vel_right=self._vel_right)
        if (cur_pos-starting) / 135 < degree:
            cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self._publisher.publish(message)
            rate.sleep()
        else:
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(message)
            return False
        return True
    
    def wait(self, rate, initial_time, duration):
        if time.time() - initial_time < duration:
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(message)
            rate.sleep()
        else:
            return False
        return True
    
    def curve(self, rate, starting, distance, direction='right'):
        cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
        if direction == 'right':
            message = WheelsCmdStamped(vel_left=self._vel_left * 2.0, vel_right=self._vel_right * 0.5)
        else:
            message = WheelsCmdStamped(vel_left=self._vel_left * 0.5, vel_right=self._vel_right * 2.0)
        if (cur_pos-starting) / 135 < distance:
            cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self._publisher.publish(message)
            rate.sleep()
        else:
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(message)
            return False
        return True
        
    def execute_initial_led_behavior(self):
        empty = ColorRGBA(r=0, g=0, b=0, a=1)
        # 1st: top left, 2nd: bottom right, 3rd: empty, 4th: bottom left, 5th: top right
        self._publisher_led.publish(LEDPattern(rgb_vals=[empty,empty,empty,empty,empty]))
        self.rate.sleep()
    
    def execute_blue_line_behavior(self):
        rgba_val = ColorRGBA(r=0, g=0, b=1, a=1)
        empty = ColorRGBA(r=0, g=0, b=0, a=1)
        # 1st: top left, 2nd: bottom right, 3rd: empty, 4th: bottom left, 5th: top right
        self._publisher_led.publish(LEDPattern(rgb_vals=[empty,rgba_val,empty,empty,rgba_val]))
        self.rate.sleep()
        self.curve(self.rate, self.starting_pos, 0.2, direction='right')
        
    def execute_red_line_behavior(self, **kwargs):
        self.drive_straight(self.rate, self.starting_pos, 0.5)
        
    def execute_green_line_behavior(self, **kwargs):
        rgba_val = ColorRGBA(r=0, g=0, b=1, a=1)
        empty = ColorRGBA(r=0, g=0, b=0, a=1)
        # 1st: top left, 2nd: bottom right, 3rd: empty, 4th: bottom left, 5th: top right
        self._publisher_led.publish(LEDPattern(rgb_vals=[rgba_val,empty,empty,rgba_val,empty]))
        self.rate.sleep()
        self.curve(self.rate, self.starting_pos, 0.25, direction='left')
        
    def callback(self, msg):
        if rospy.is_shutdown():
            self.stop_wheels()

        if self.stop:
            if time.time() - self.start_time < 3:
                self.stop_wheels()
                return
            else:
                self.stop = False
                self.starting_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)

        if self.state != 'regular':
            idx = ['red', 'blue', 'green'].index(self.state)
            [self.execute_red_line_behavior, self.execute_blue_line_behavior, self.execute_green_line_behavior][idx]()
            return

        if self._ticks_right is None or self._ticks_left is None or self._starting_ticks_left is None:
            return

        if not self.stop:
            starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self.drive_straight(self.rate, starting, 2)
            # add your code here
            imageFrame = self._bridge.compressed_imgmsg_to_cv2(msg)

            height, width, _ = imageFrame.shape

            hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
            red_mask = cv2.inRange(hsvFrame, self.red_lower, self.red_upper)
            green_mask = cv2.inRange(hsvFrame, self.green_lower, self.green_upper)
            blue_mask = cv2.inRange(hsvFrame, self.blue_lower, self.blue_upper)

            kernel = np.ones((5, 5), "uint8")

            red_mask = cv2.dilate(red_mask, kernel)
            res_red = cv2.bitwise_and(imageFrame, imageFrame, mask = red_mask)

            green_mask = cv2.dilate(green_mask, kernel)
            res_green = cv2.bitwise_and(imageFrame, imageFrame, mask = green_mask)

            blue_mask = cv2.dilate(blue_mask, kernel)
            res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask = blue_mask)

        if not self.stop:
            contours, hierarchy = cv2.findContours(red_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
            for point in contours:
                x, y = point[0][0]
                if y >= 0.9 * height:
                    self.stop = True
                    self.start_time = time.time()
                    self.state = 'red'
                    break

        if not self.stop:
            contours, hierarchy = cv2.findContours(green_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
            for point in contours:
                x, y = point[0][0]
                if y >= 0.9 * height:
                    self.stop = True
                    self.start_time = time.time()
                    self.state = 'green'
                    break

        if not self.stop:
            contours, hierarchy = cv2.findContours(blue_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
            for point in contours:
                x, y = point[0][0]
                if y >= 0.9 * height:
                    self.stop = True
                    self.start_time = time.time()
                    self.state = 'blue'
                    break
            
        
        
        # colors_detected_msg = self._bridge.cv2_to_imgmsg(imageFrame, encoding="bgr8")

        # self._publisher_image.publish(colors_detected_msg)

        # self.set_led_pattern(self, colors)

        self.rate.sleep()

    # add other functions as needed
    def stop_wheels(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    node = BehaviorController(node_name='behavior_controller_node')
    # node.run()
    rospy.spin()