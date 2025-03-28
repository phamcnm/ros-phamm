#!/usr/bin/env python3

# # potentially useful for question - 1.1 - 1.4 and 2.1

# # import required libraries
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
import yaml

THROTTLE_LEFT = 0.12
DIRECTION_LEFT = 1
THROTTLE_RIGHT = 0.135
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
    
class PDController:
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.previous_error = 0
        self.delta = 5

    def compute(self, error, dt):
        error = min(error, self.previous_error + self.delta)
        error = max(error, self.previous_error - self.delta)
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Kd * derivative
        self.previous_error = error
        return output

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        self.delta = 5

    def compute(self, error, dt):
        error = min(error, self.previous_error + self.delta)
        error = max(error, self.previous_error - self.delta)
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

class LaneDetectionNode(DTROS):
    def __init__(self, node_name):
        super(LaneDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.side = 0 # 0 for left, 1 for right

        # add your code here
        self._bridge = CvBridge()
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        # construct subscriber
        camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub = rospy.Subscriber(camera_topic, CompressedImage, self.callback, queue_size=1)
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

        # Load the YAML file
        with open("./packages/cv_undistort/src/csc22936.yaml", "r") as file:
            calib_data = yaml.safe_load(file)

        # Extract the camera matrix (K)
        self.K = np.array(calib_data["camera_matrix"]["data"]).reshape(3, 3)

        # Extract distortion coefficients (D)
        self.D = np.array(calib_data["distortion_coefficients"]["data"])

        self.image = None

        self.rate_camera = rospy.Rate(100)
        self.rate_wheel = rospy.Rate(100)
        # self.rate_contours = rospy.Rate(3)

        self.h = None
        self.w = None

        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        # temporary data storage
        self._ticks_left = None
        self._ticks_right = None
        self._starting_ticks_left = None
        self._starting_ticks_right = None
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        # self._publisher_signal = rospy.Publisher(signal_topic, WheelsCmdStamped, queue_size=1)
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT 
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        self._radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 100)

        # self.white_lower = np.array([0, 0, 170], np.uint8)
        # self.white_upper = np.array([180, 25, 255], np.uint8)
        # self.yellow_lower = np.array([20, 120, 120], np.uint8)
        # self.yellow_upper = np.array([33, 255, 255], np.uint8)

        #old config
        self.white_lower = np.array([0, 0, 170], np.uint8)
        self.white_upper = np.array([180, 25, 255], np.uint8)
        self.yellow_lower = np.array([22, 120, 120], np.uint8)
        self.yellow_upper = np.array([28, 255, 255], np.uint8)
        self._publisher_image = rospy.Publisher(f"/lane", Image, queue_size=1)

        self.counter = 0

        # phasing and auxiliary variables
        self.waiting = False
        self.default_waiting_time = 2
        self.performing_turn = False
        self.detecting_corner = False
        self.vision_detected = False

        self.starting_time = None
        self.starting_location = None


        # camera calibration parameters (intrinsic matrix and distortion coefficients)
        
        # color detection parameters in HSV format
        
        # initialize bridge and subscribe to camera feed

        # lane detection publishers

        # LED
        
        # ROI vertices
        
        # define other variables as needed
        self.image = None
        self.p = PController(Kp=0.5)
        self.pd = PDController(Kp=0.5, Kd=0.1)
        self.pid = PIDController(Kp=0.5, Ki=0.01, Kd=0.1)

        self.white_contours = None
        self.yellow_contours = None

        self.prev_steering = 0
        self.delta = 0.3

        # self.stop_wheels()

    def undistort_image(self, **kwargs):
        # add your code here
        pass

    def preprocess_image(self, **kwargs):
        # add your code here
        pass
    
    def detect_lane_color(self, **kwargs):
        # add your code here
        pass
    
    def detect_lane(self, **kwargs):
        # add your code here
        # potentially useful in question 2.1
        pass

    def set_steering(self, steering):
        mult = 0.32
        print('In set steering: %f' % steering)
        message = WheelsCmdStamped(vel_left=(1-mult*steering)*self._vel_left, vel_right= (1+mult*steering)*self._vel_right)
        self._publisher.publish(message)
        self.rate_wheel.sleep()

    def drive_straight(self, rate):
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        self._publisher.publish(message)
        rate.sleep()

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

    def stop_wheels(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

    def detect_failure(self, white_contours, yellow_contours, side):
        if len(yellow_contours) == 0 and len(white_contours) == 0:
            print('Failure Mode 0: seeing no white no yellow. WTF!')
            return True
        
        # count_whites, count_yellows = 0, 0
        # for point in white_contours:
        #     x, y = point[0][0]
        #     if y >= 0.5 * self.h: # ignore anything far ahead
        #         count_whites += 1

        # for point in yellow_contours:
        #     x, y = point[0][0]
        #     if y >= 0.5 * self.h: # ignore anything far ahead
        #         count_yellows += 1

        # if count_whites == 0 and count_yellows == 0:
        #     print('Failure Mode 1: seeing no white no yellow near bottom. WTF!')
        #     return True

        return False

    def detect_corner(self, white_contours, yellow_contours, side):
        if side == 0: # left
            if len(yellow_contours) == 0 and len(white_contours) > 0:
                return True
        if side == 1:
            whites_in_bottom = False
            for point in white_contours:
                x, y = point[0][0]
                if y >= 0.9 * self.h:
                    whites_in_bottom = True
                    break
            if whites_in_bottom is False and len(yellow_contours) > 0:
                print('Detected corner!')
                return True
        return False


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
            return False # not yet done
        return True # done rotating
    
    def wait(self, rate, initial_time, duration):
        if time.time() - initial_time < duration:
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(message)
            rate.sleep()
        else:
            return True
        return False


    
    # def run(self):
    #     self.stop_wheels()

    #     while not rospy.is_shutdown():
    #         if self._ticks_right is None or self._ticks_left is None or self._starting_ticks_left is None:
    #             continue
            
    #         self.drive_straight(self.rate_wheel)

    #         if self.waiting:
    #             print('in waiting')
    #             done = self.wait(self.rate_wheel, starting_time, self.default_waiting_time)
    #             if done:
    #                 self.waiting = False
    #             else:
    #                 continue

    #         if self.performing_turn:
    #             print('in performing turn')
    #             direction = ['left', 'right'][1-self.side]
    #             done = self.rotate(self.rate_wheel, starting_location, 0.03, direction=direction)
    #             if done:
    #                 self.performing_turn = False
    #             else:
    #                 continue

    #         if not self.vision_detected:
    #             continue
            
    #         # vision detected => maneuver
    #         self.detecting_failure = self.detect_failure(self.white_contours, self.yellow_contours, self.side)
    #         if self.detecting_failure:
    #             self.stop_wheels()
    #             rospy.signal_shutdown("Finished")
    #             break

    #         self.detecting_corner = self.detect_corner(self.white_contours, self.yellow_contours, self.side)
    #         if self.detecting_corner:
    #             starting_time = time.time()
    #             self.waiting = True
    #             self.detecting_corner = False
    #             self.performing_turn = True
    #             starting_location = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
    #             continue

    #         dt = 0.33  # Time step
    #         error = self.get_lane_error(self.white_contours, self.yellow_contours, self.side)
    #         steering_angle = self.pid.compute(error, dt)
    #         self.set_steering(steering_angle)
    #         self.vision_detected = False

    #         continue

    #     self.stop_wheels()
    #     print('Outside of main loop')
    #     # rospy.signal_shutdown("Finished")

    
    def callback(self, msg):
        print(self.counter, ' in callback')

        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        self.h, self.w = image.shape[:2]

        image = image[int(0.5 * self.h):, :]
        self.h, self.w = image.shape[:2]

        new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (self.w, self.h), alpha=1, centerPrincipalPoint=True)

        undistorted_img = cv2.undistort(image, self.K, self.D, None, new_K)

        hsvFrame = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2HSV) 
        white_mask = cv2.inRange(hsvFrame, self.white_lower, self.white_upper)
        yellow_mask = cv2.inRange(hsvFrame, self.yellow_lower, self.yellow_upper)

        kernel = np.ones((3, 3), "uint8")

        white_mask = cv2.dilate(white_mask, kernel)
        res_white = cv2.bitwise_and(undistorted_img, undistorted_img, mask = white_mask)

        yellow_mask = cv2.dilate(yellow_mask, kernel)
        res_yellow = cv2.bitwise_and(undistorted_img, undistorted_img, mask = yellow_mask)

        white_contours, white_hierarchy = cv2.findContours(white_mask, 
                                        cv2.RETR_TREE, 
                                        cv2.CHAIN_APPROX_SIMPLE)

        yellow_contours, yellow_hierarchy = cv2.findContours(yellow_mask, 
                                        cv2.RETR_TREE, 
                                        cv2.CHAIN_APPROX_SIMPLE)
        
        self.white_contours = white_contours
        self.yellow_contours = yellow_contours
        
        if self._ticks_right is None or self._ticks_left is None or self._starting_ticks_left is None:
            return
        
        # self.drive_straight(self.rate_wheel)

        # if self.waiting:
        #     print('in waiting')
        #     done = self.wait(self.rate_wheel, self.starting_time, self.default_waiting_time)
        #     if done:
        #         self.waiting = False
        #     else:
        #         return

        if self.performing_turn:
            print('in performing turn')
            direction = ['left', 'right'][1-self.side]
            done = self.rotate(self.rate_wheel, self.starting_location, 0.03, direction=direction)
            if done:
                self.performing_turn = False
            else:
                return

        # if not self.vision_detected:
        #     return
        
        # vision detected => maneuver
        self.detecting_failure = self.detect_failure(self.white_contours, self.yellow_contours, self.side)
        # if self.detecting_failure:
        #     self.detecting_corner = True
            # self.stop_wheels()
            # rospy.signal_shutdown("Finished")
            # return

        self.detecting_corner = self.detect_corner(self.white_contours, self.yellow_contours, self.side)
        if self.detecting_failure or self.detecting_corner:
            self.starting_time = time.time()
            self.waiting = True
            self.detecting_corner = False
            self.performing_turn = True
            self.starting_location = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            return

        dt = 0.01  # Time step
        error = self.get_lane_error(self.white_contours, self.yellow_contours, self.side)
        steering_angle = self.pd.compute(error, dt)
        
        steering_angle = min(steering_angle, self.prev_steering + self.delta)
        steering_angle = max(steering_angle, self.prev_steering - self.delta)
        self.prev_steering = steering_angle
        self.set_steering(steering_angle)
        # self.vision_detected = False

        # anything else you want to add here
        # self.image = undistorted_img

        # for pic, contour in enumerate(white_contours): 
        #     area = cv2.contourArea(contour) 
        #     if(area > 300): 
        #         x, y, w, h = cv2.boundingRect(contour) 
        #         undistorted_img = cv2.rectangle(undistorted_img, (x, y), 
        #                                 (x + w, y + h), 
        #                                 (0, 255, 0), 2) 
                
        #         cv2.putText(undistorted_img, "White Colour", (x, y), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 
        #                     1.0, (0, 255, 0)) 

        # for pic, contour in enumerate(yellow_contours): 
        #     area = cv2.contourArea(contour) 
        #     if(area > 300): 
        #         x, y, w, h = cv2.boundingRect(contour) 
        #         undistorted_img = cv2.rectangle(undistorted_img, (x, y), 
        #                                 (x + w, y + h), 
        #                                 (0, 0, 255), 2) 
                
        #         cv2.putText(undistorted_img, "Yellow Colour", (x, y), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 
        #                     1.0, (0, 0, 255))
                
        
                
        undistorted_msg = self._bridge.cv2_to_imgmsg(undistorted_img, encoding="bgr8")

        self._publisher_image.publish(undistorted_msg)

        # self.vision_detected = True
        self.counter += 1

        self.rate_camera.sleep()

    # add other functions as needed

if __name__ == '__main__':
    node = LaneDetectionNode(node_name='lane_detection_node')
    # node.run()
    rospy.spin()
