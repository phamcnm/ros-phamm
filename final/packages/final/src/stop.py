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
THROTTLE_RIGHT = 0.355

class PController:
    def __init__(self, Kp):
        self.Kp = Kp
        self.previous_error = 0
        self.delta = 5

    def compute(self, error, dt=0.01):
        error = min(error, self.previous_error + self.delta)
        error = max(error, self.previous_error - self.delta)
        output = self.Kp * error
        self.previous_error = error
        return output

class StopNode(DTROS):

    def __init__(self, node_name):
        super(StopNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # for undistort
        with open("./packages/final/src/csc22936.yaml", "r") as file:
            calib_data = yaml.safe_load(file)
        self.K = np.array(calib_data["camera_matrix"]["data"]).reshape(3, 3)
        self.D = np.array(calib_data["distortion_coefficients"]["data"])
        
        # topics
        camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        signal_topic = f"/{self._vehicle_name}/led_emitter_node/led_pattern"

        # publishers
        self._publisher_camera = rospy.Publisher('poster', Image, queue_size=1)
        self._publisher_wheel = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self._publisher_led = rospy.Publisher(signal_topic, LEDPattern, queue_size=1)

        # camera
        self._bridge = CvBridge()
        self.sub = rospy.Subscriber(camera_topic, CompressedImage, self.camera_callback)

        # odometry
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)

        self._ticks_left = None
        self._starting_ticks_left = None
        self._radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 100)
        self.rate = rospy.Rate(20)

        self._vel_left, self._vel_right = THROTTLE_LEFT, THROTTLE_RIGHT

        # pre-run
        self.stop_wheels()

        
    def callback_left(self, data):
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        self._ticks_left = data.data
        if not self._starting_ticks_left:
            self._starting_ticks_left = self._ticks_left

    def publish_leds(self, colors):
        rgba_val = ColorRGBA(r=colors[0], g=colors[1], b=colors[2], a=1)
        self._publisher_led.publish(LEDPattern(rgb_vals=[rgba_val,rgba_val,rgba_val,rgba_val,rgba_val]))


    def get_lane_error(self, white_contours, yellow_contours):
        max_white_point = (0, 0)
        max_yellow_point = (self.w, 0)

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
        prop = (cur - avg) / self.w
        return prop
    
    def set_steering(self, steering, speed = 1, mult = 0.4):
        print('In set steering: %f' % steering)
        message = WheelsCmdStamped(vel_left=(1-mult*steering)*self._vel_left * speed, vel_right= (1+mult*steering)*self._vel_right * speed)
        self._publisher_wheel.publish(message)

    def drive_straight(self, starting, distance, mult_left=1, mult_right=1):
        message = WheelsCmdStamped(vel_left=mult_left * self._vel_left, vel_right=mult_right * self._vel_right)
        cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
        if (cur_pos - starting) / 135 < distance:
            cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self._publisher_wheel.publish(message)
            self.rate.sleep()
            return False # not yet done
        else:
            self.stop_wheels()
            return True # done, got to distance
        
    
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
    
    def curve(self, starting, distance, direction='right'):
        cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
        if direction == 'right':
            message = WheelsCmdStamped(vel_left=self._vel_left * 2.0, vel_right=self._vel_right * 0.3)
        else:
            message = WheelsCmdStamped(vel_left=self._vel_left * 0.5, vel_right=self._vel_right)
        if (cur_pos-starting) / 135 < distance:
            cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self._publisher_wheel.publish(message)
            self.rate.sleep()
            return False
        else:
            return True

    
    def stop_wheels(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher_wheel.publish(stop)

    def wait(self, duration=1):
        if time.time() - self.start_time < duration:
            self.stop_wheels()
            return False # not yet done
        else:
            self.phase = self.after_phase
            self.starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            return True # done, duration has passed.
        

    def next_phase(self):
        self.phase += 1
        self.start_time = time.time()
        self.start_location = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)

    def get_grid_dimensions(self, corners):
        # corners is the array returned by cv2.findCirclesGrid
        
        # Find the minimum and maximum x,y coordinates
        min_x = np.min(corners[:, 0, 0])
        max_x = np.max(corners[:, 0, 0])
        min_y = np.min(corners[:, 0, 1])
        max_y = np.max(corners[:, 0, 1])
        
        # Calculate width and height
        width = max_x - min_x
        height = max_y - min_y
        
        return width, height


    def camera_callback(self, msg):
        if rospy.is_shutdown():
            self.stop_wheels()

        if time.time() - self.timer < 0.3:
            return

        if self._ticks_left is None or self._starting_ticks_left is None:
            return
        
        if not self.starting:
            self.starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
        # undistort image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # self.h, self.w = image.shape[:2]
        # image = image[:int(0.5 * self.h), :]
        self.h, self.w = image.shape[:2]
        new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (self.w, self.h), alpha=1, centerPrincipalPoint=True)
        undistorted_img = cv2.undistort(image, self.K, self.D, None, new_K)

        stopped = 0

        if self.redline_cooldown > 0 and time.time() - self.redline_cooldown > 7:
            self.redline_cooldown = -1

        print(self.phase)
        if self.phase == 'waiting':
            print('in waiting')
            self.wait()

        hsvFrame = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsvFrame, self.white_lower, self.white_upper)
        yellow_mask = cv2.inRange(hsvFrame, self.yellow_lower, self.yellow_upper)
        red_mask = cv2.inRange(hsvFrame, self.red_lower, self.red_upper)
        kernel = np.ones((3, 3), "uint8")

        white_contours, white_hierarchy = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        yellow_contours, yellow_hierarchy = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        red_contours, red_hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        speed_mult = 1
        is_found, centers = cv2.findCirclesGrid(undistorted_img, (7, 3), flags=cv2.CALIB_CB_SYMMETRIC_GRID)
        if is_found or centers is not None:
            grid_width, grid_height = self.get_grid_dimensions(centers)
            if grid_width > 70 or grid_height > 25:
                speed_mult = 0.2
            elif grid_width > 90 or grid_height > 40:
                speed_mult = 0.05

        if self.stage == 1:
            if self.phase == 'tailing' and stopped < 3:
                error = self.get_lane_error(white_contours, yellow_contours)
                steering_angle = self.p.compute(error)
                steering_angle = min(steering_angle, self.prev_steering + self.delta)
                steering_angle = max(steering_angle, self.prev_steering - self.delta)
                self.prev_steering = steering_angle
                self.set_steering(steering_angle, speed=speed_mult)

                if self.redline_cooldown < 0:
                    for red_point in red_contours:
                        x, y = red_point[0][0]
                        if y >= 0.7 * self.h:
                            print('detecting red')
                            self.start_time = time.time()
                            self.after_phase = self.after_phases[self.stopped]
                            self.phase = 'waiting'
                            self.redline_cooldown = time.time()
                            stopped += 1
                            break

                self.starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)

            elif self.phase == 'right_rurn':
                done = self.curve(self.starting, 0.2, 'right')
                if done:
                    self.phase = 'tailing'
                else:
                    return

            elif self.phase == 'left_turn':
                done = self.curve(self.starting, 0.3, 'left')
                if done:
                    self.phase = 'last'
                else:
                    return
            elif self.phase == 'last':

                error = self.get_lane_error(white_contours, yellow_contours)
                steering_angle = self.p.compute(error)
                steering_angle = min(steering_angle, self.prev_steering + self.delta)
                steering_angle = max(steering_angle, self.prev_steering - self.delta)
                self.prev_steering = steering_angle
                self.set_steering(steering_angle, speed=speed_mult)

                if self.redline_cooldown < 0:
                    for red_point in red_contours:
                        x, y = red_point[0][0]
                        if y >= 0.7 * self.h:
                            print('detecting red')
                            self.start_time = time.time()
                            #change this
                            self.after_phase = 'start'
                            self.stage += 1
                            self.phase = 'waiting'
                            self.redline_cooldown = time.time()
                            stopped += 1
                            break

        self.timer = time.time()


if __name__ == '__main__':
    node = StopNode(node_name='safe_navigator')
    rospy.spin()
