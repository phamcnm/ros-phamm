#!/usr/bin/env python3

import rospy
import sys
import argparse
import os
import math
import numpy as np
import cv2
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CameraInfo, CompressedImage, Range
from std_msgs.msg import Float32, ColorRGBA
from turbojpeg import TurboJPEG
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, LEDPattern
import threading, time
from dt_apriltags import Detector
from collections import deque
from geometry_msgs.msg import Point32


ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
RED_MASK = [(136, 87, 111), (180, 255, 255)]
BLUE_MASK = [(100, 80, 50), (130, 255, 255)]

DEBUG = False
ENGLISH = False
SAFETY = False
AUSSIE = False

class BlueTurnDetector:
    def __init__(self, history_len=45):
        self.prev_centers = deque(maxlen=history_len)  # store past center x positions

    def detect_blue_turn(self, img):
        print('in detect blue turn')
        hsv_full = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_full, np.array(BLUE_MASK[0]), np.array(BLUE_MASK[1]))
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        large_blue_contours = [cnt for cnt in blue_contours if cv2.contourArea(cnt) > 200]

        if not large_blue_contours:
            self.prev_centers.append(None)
            return None

        largest = max(large_blue_contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            self.prev_centers.append(None)
            return None

        cx = int(M["m10"] / M["m00"])
        print(f"cx {cx}")
        self.prev_centers.append(cx)
        return None  # only make decision in next_phase()

    def get_direction(self):
        valid_centers = [v for v in self.prev_centers if v is not None]
        if len(valid_centers) < 1:
            return "LEFT"  # Not enough data

        values = np.array(valid_centers)
        weights = np.exp(np.linspace(0, 2, len(values)))  # or linear: np.linspace(0.1, 1.0, ...)
        weights /= np.sum(weights)

        weighted_avg = np.dot(values, weights)
        baseline_avg = np.mean(values)

        if weighted_avg - baseline_avg > 0:
            return "RIGHT"
        else:
            return "LEFT"
    
class LaneFollowNode(DTROS):
    def __init__(self, node_name, zone=1):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = os.environ['VEHICLE_NAME']

        self._init_logic_params(zone)
        
        # Initialize TurboJPEG for image decoding
        self.jpeg = TurboJPEG()
        
        # Set up publishers and subscribers
        self._setup_publishers_subscribers()
        
        # Initialize control parameters
        self._init_control_params()

        self.last_stamp = rospy.Time.now()

        self.cbParametersChanged()

        self.detector = BlueTurnDetector()

        #stage two stuff
        self.at_detector = Detector(searchpath=['apriltags'],
                families='tag36h11',
                nthreads=1,
                quad_decimate=2.0,
                quad_sigma=0.0,
                refine_edges=0,
                decode_sharpening=0.25,
                debug=0)
        self.stage2_turn = "left"
        
        # Wait before sending motor commands
        rospy.Rate(0.20).sleep()
        self.publish_leds([0,0,0])
        
        # Shutdown hook
        rospy.on_shutdown(self.hook)
        
        self.loginfo("Initialized")

    def cbParametersChanged(self):
        process_frequency = 5
        self.publish_duration = rospy.Duration.from_sec(1.0 / process_frequency)
        params = cv2.SimpleBlobDetector_Params()

        # Allow small blobs, even noisy ones
        params.filterByArea = True
        params.minArea = 10   # very small blobs, more sensitive
        params.maxArea = 8000  # generous upper bound in case of partial lighting

        # Allow slightly misshapen blobs
        params.filterByCircularity = False
        params.filterByInertia = False
        params.filterByConvexity = False

        # Detect black blobs (important!)
        params.filterByColor = True
        params.blobColor = 0  # black

        # Allow close blobs to be treated individually
        params.minDistBetweenBlobs = 2
        self.simple_blob_detector = cv2.SimpleBlobDetector_create(params)
        self.circlepattern_dims = [7, 3]

    def publish_leds(self, colors):
        empty = ColorRGBA(r=0, g=0, b=0, a=1)
        rgba_val = ColorRGBA(r=colors[0], g=colors[1], b=colors[2], a=1)
        self._publisher_led.publish(LEDPattern(rgb_vals=[empty,rgba_val,rgba_val,rgba_val,empty]))

    def detect_peDuckstrians(self, img):
        crop = img[250:-1, 250:-1, :]
        hsvFrame = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        self.orange_lower = np.array([9, 91, 163], np.uint8)
        self.orange_upper = np.array([22, 255, 255], np.uint8)

        orange_mask = cv2.inRange(hsvFrame, self.orange_lower, self.orange_upper)
        
        contours, hierarchy = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

        for c in contours:
            print(f"cv2.contourArea(cnt) {cv2.contourArea(c)}")

        return len(contours) > 0

    def _setup_publishers_subscribers(self):
        """Set up all publishers and subscribers"""
        # Publishers
        self.pub = rospy.Publisher(f"/{self.veh}/output/image/mask/compressed",
                                  CompressedImage, queue_size=1)
        self.pub_curve_detection = rospy.Publisher(f"/{self.veh}/output/image/curve_detection/compressed",
                                                  CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher(f"/{self.veh}/car_cmd_switch_node/cmd",
                                      Twist2DStamped, queue_size=1)
        
        # Subscribers
        self.sub = rospy.Subscriber(f"/{self.veh}/camera_node/image/compressed",
                                   CompressedImage, self.callback, 
                                   queue_size=1, buff_size="20MB")
        
        self._publisher_led = rospy.Publisher(f"/{self.veh}/led_emitter_node/led_pattern", LEDPattern, queue_size=1)
        
        # Conditional subscribers
        if SAFETY:
            self.tof_sub = rospy.Subscriber(f"/{self.veh}/front_center_tof_driver_node/range",
                                           Range, self.cb_tof, queue_size=1)

    def _init_control_params(self):
        """Initialize control parameters"""
        # PID Variables
        self.proportional = None
        self.last_error = 0
        self.integral = 0
        self.last_time = rospy.get_time()
        
        # Position offset based on configuration
        if ENGLISH:
            self.offset = -180
        elif AUSSIE:
            self.offset = 0
        else:
            self.offset = 230
            
        # Velocity control
        self.base_velocity = 0.2
        self.velocity = self.base_velocity
        self.twist = Twist2DStamped(v=self.velocity, omega=0)
        
        # Memory for lane loss handling
        self.last_valid_omega = 0
        self.last_valid_velocity = self.base_velocity
        self.no_contour_count = 0
        self.max_no_contour_frames = 10
        
        # PID coefficients
        if AUSSIE:
            self.P = 0.0005
            self.D = -0.025
            self.I = 0.5
        else:
            self.P = 0.025
            self.D = -0.0025
            self.I = 0
            
        # Turn enhancement parameters
        self.curve_coefficient = 0.02
        self.angle_coefficient = 0.3
        self.large_error_threshold = 150
        self.turn_boost_threshold = 50
        self.right_turn_threshold = 35
        self.turn_boost_factor = 1.6
        self.max_velocity = 0.6
        self.omega_multiplier = 1.5
        self.omega_right_multiplier = 2.0
        
        # State variables
        self.curvature = 0
        self.curve_detection = False
        self.multiple_points = []
        self.tof_distance = 1.0
        self.obj_stop = False
        self.duckie_stop = False
        self.lock = threading.Lock()
        self.duckie_stop = False

        #part 3
        self.seen_line = 0
        self.blueline_start_time = None
        self.blueline_stop = False
        self.disable_drive = False
        self.last_disable_drive = False
        self.manuvering = False
        self.manuver_state = 0
        self.state_time = 0
        self.blue_count = 0
        self.see_blue_timer = time.time()
        self.tailing_timer = time.time()
        self.seen_blue = False
        self.see_peduckstrian_timer = time.time()
        self.seen_peduckstrian = False

        #part 4
        self.park_timer = time.time()
        self.tag_timer = time.time()
        self.reset_timer = time.time()
        self.zone_count = 0

    def _init_logic_params(self, zone):
        self.phase = 0
        self.red_cooldown_until = 0
        # self.tailing_cooldown = 0
        self.current_leds = [0,0,0]
        self.max_phase = 0
        self.stage1_phases = ['tailing', 'right_turn', 'straight', 'left_turn']

        #stage 2 stuff
        self.stage = 4

        self.stage2_phases = ['default', 'right', 'left']
        self.seen_tag = False
        self.driving = True

        #stage 3 stuff
        self.stage3_phases = ['default', 'end']
        self.blue_timer = time.time()

        #stage 4
        self.parking = False
        self.zone = zone
        self.stage4_phases = ['park', 'follow']

    def cb_tof(self, msg):
        """Process Time-of-Flight sensor data"""
        self.tof_distance = msg.range
        if 0.05 < self.tof_distance <= 0.3:
            self.obj_stop = True

    def _process_image(self, crop, img=None):
        """Process image to find lane and calculate proportional error"""
        crop_width = crop.shape[1]
        crop_height = crop.shape[0]
        
        # Convert to HSV and create mask for yellow lane
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])

        if self.stage != 4:
            if time.time() > self.red_cooldown_until:
                red_mask = cv2.inRange(hsv, RED_MASK[0], RED_MASK[1])
                red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                    
                for i, contour in enumerate(red_contours):
                    if cv2.contourArea(contour) < 100:
                        continue
                    for point in contour:
                        x, y = point[0]
                        if y >= 0.7 * crop_height:
                            print("red detected") 
                            return None, [], True, False, None
                    
        now = rospy.Time.now()
        if (self.stage == 1 or self.stage == 2) and self.phase == 0 and now - self.last_stamp > self.publish_duration:
            self.last_stamp = now

            # detect the first turn
            if self.phase == 0 and self.max_phase == 0 and img is not None:
                self.detector.detect_blue_turn(img)

            blue_mask = cv2.inRange(hsv, BLUE_MASK[0], BLUE_MASK[1])
            blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            large_blue_contours = [cnt for cnt in blue_contours if cv2.contourArea(cnt) > 200]
            num_large_blue = len(large_blue_contours)
            
            if num_large_blue > 0:
                print(cv2.contourArea(large_blue_contours[0]))
                print('seen blue', num_large_blue)
                self.disable_drive = True
                self.tailing_timer = time.time()
                if self.current_leds != [1, 0, 0]:
                    self.publish_leds([1, 0, 0])
                    self.current_leds = [1, 0, 0]
                return None, [], False, True, blue_mask
            else:
                if self.current_leds != [0, 0, 0] and time.time() - self.tailing_timer > 15:
                    self.publish_leds([0, 0, 0])
                    self.current_leds = [0, 0, 0]
                self.disable_drive = False

        if self.stage == 3:
            if now - self.last_stamp > self.publish_duration:
                self.last_stamp = now
            
            if self.blue_count == 1 or self.blue_count == 4: # seen blue line, but waiting for pedestrian
                print("blue detected at blue count, waiting!!!!,", self.blue_count)
                return None, [], False, True, None
            elif time.time() - self.blue_timer > 5:
                if self.blue_count == 0 or self.blue_count == 3: 
                    blue_mask = cv2.inRange(hsv, BLUE_MASK[0], BLUE_MASK[1])
                    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                        
                    for i, contour in enumerate(blue_contours):
                        if cv2.contourArea(contour) < 250:
                            continue
                        for point in contour:
                            x, y = point[0]
                            if y >= 0.7 * crop_height:
                                print("blue detected at blue count", self.blue_count)
                                self.blue_count += 1
                                self.see_peduckstrian_timer = time.time()
                                self.disable_drive = True
                                return None, [], False, True, None
                elif self.blue_count == 2: # looking for broken bot
                    blue_mask = cv2.inRange(hsv, BLUE_MASK[0], BLUE_MASK[1])
                    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                    large_blue_contours = [cnt for cnt in blue_contours if cv2.contourArea(cnt) > 100]
                    num_large_blue = len(large_blue_contours)

                    if num_large_blue > 0:
                        print(cv2.contourArea(large_blue_contours[0]))
                        # print('seen blue', num_large_blue)
                        self.disable_drive = True
                        return None, [], False, True, blue_mask

                
        max_area = 20
        max_idx = -1
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > max_area:
                max_idx = i
                max_area = area
                
        # Process contour if found
        multiple_points = []
        proportional = None
        
        if max_idx != -1:
            try:
                # Calculate center of contour
                M = cv2.moments(contours[max_idx])
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                proportional = cx - int(crop_width / 2) + self.offset
                
                # Divide the mask into horizontal sections for multi-point analysis
                sections = 3
                section_height = crop_height // sections
                
                for i in range(sections):
                    y_start = i * section_height
                    y_end = (i+1) * section_height
                    section_mask = mask[y_start:y_end, :]
                    section_contours, _ = cv2.findContours(section_mask, 
                                                          cv2.RETR_EXTERNAL, 
                                                          cv2.CHAIN_APPROX_NONE)
                    
                    # Find largest contour in this section
                    max_section_area = 10
                    max_section_idx = -1
                    
                    for j, contour in enumerate(section_contours):
                        section_area = cv2.contourArea(contour)
                        if section_area > max_section_area:
                            max_section_idx = j
                            max_section_area = section_area
                    
                    if max_section_idx != -1:
                        try:
                            section_M = cv2.moments(section_contours[max_section_idx])
                            section_cx = int(section_M['m10'] / section_M['m00'])
                            section_cy = int(section_M['m01'] / section_M['m00'])
                            multiple_points.append((section_cx, y_start + section_height//2))
                            
                            # Draw points for debugging
                            if DEBUG:
                                cv2.circle(crop, (section_cx, y_start + section_height//2), 5, (255, 0, 0), -1)
                        except:
                            pass
                
                # Draw contour and center for debugging
                if DEBUG:
                    cv2.drawContours(crop, contours, max_idx, (0, 255, 0), 3)
                    cv2.circle(crop, (cx, cy), 7, (0, 0, 255), -1)
                    
            except Exception as e:
                self.logwarn(f"Error in contour processing: {e}")
        
        return proportional, multiple_points, False, False, None
    
    def _calculate_pid(self, proportional):
        """Calculate PID control values based on proportional error"""
        # Adjust PID parameters based on error magnitude
        if abs(proportional) > self.large_error_threshold:
            effective_P = self.P * 1.5
            effective_D = self.D * 1.2
        else:
            effective_P = self.P
            effective_D = self.D
        
        # P Term
        P = -proportional * effective_P

        # D Term
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        if dt <= 0:
            D = 0
        else:
            d_error = (proportional - self.last_error) / dt
            D = d_error * effective_D
        
        # I term
        if dt <= 0:
            I = 0
        else:
            self.integral += proportional * dt
            # Reset integral when crossing the center to prevent oscillation
            if (proportional * self.last_error) < 0:
                self.integral = 0
            I = self.I * self.integral
        
        # Update state variables
        self.last_error = proportional
        self.last_time = current_time
        
        # Calculate basic PID output
        pid_output = P + D + I
        
        return pid_output, P, D, I
    
    def _calculate_steering(self, pid_output, proportional):
        """Calculate final steering command with turn enhancements"""
        # Add angle-based adjustment from multiple points
        angle_adjustment = 0
        if len(self.multiple_points) >= 2:
            # Calculate angle of line from first to last point
            dy = self.multiple_points[-1][1] - self.multiple_points[0][1]
            dx = self.multiple_points[-1][0] - self.multiple_points[0][0]
            if dy != 0:  # Avoid division by zero
                angle = math.atan2(dx, dy)  # Using atan2 to get the correct quadrant
                angle_adjustment = self.angle_coefficient * angle
        
        # Combine steering components
        base_omega = pid_output + angle_adjustment
        
        # Determine turn direction and enhance steering if needed
        turning_right = proportional > 0
        turning_left = proportional < 0
        
        in_right_turn = turning_right and abs(proportional) > self.right_turn_threshold
        in_left_turn = turning_left and abs(proportional) > self.turn_boost_threshold
        
        if in_right_turn:
            # Enhance right turns
            omega = base_omega * self.omega_right_multiplier
            boost_factor = min(1.0, abs(proportional) / 200.0)
            velocity = min(self.max_velocity, self.base_velocity * (1.0 + boost_factor * (self.turn_boost_factor - 1.0)))
            
            if DEBUG:
                self.loginfo(f"RIGHT TURN (error={proportional:.1f}) - Omega multiplier: {self.omega_right_multiplier:.1f}, Speed boost: {velocity:.2f}")
                
        elif in_left_turn:
            # Enhance left turns
            omega = base_omega * self.omega_multiplier
            boost_factor = min(1.0, abs(proportional) / 200.0)
            velocity = min(self.max_velocity, self.base_velocity * (1.0 + boost_factor * (self.turn_boost_factor - 1.0)))
            
            if DEBUG:
                self.loginfo(f"LEFT TURN (error={proportional:.1f}) - Omega multiplier: {self.omega_multiplier:.1f}, Speed boost: {velocity:.2f}")
                
        else:
            # Standard steering
            omega = base_omega
            velocity = self.base_velocity
            
        return omega, velocity, angle_adjustment

    def _handle_no_contour(self):
        """Handle case when no lane is detected"""
        self.no_contour_count += 1
        
        if self.no_contour_count <= self.max_no_contour_frames:
            # Continue with last valid command for a short period
            omega = self.last_valid_omega
            velocity = self.last_valid_velocity
            
            if DEBUG:
                self.loginfo(f"NO CONTOUR - Using last values: omega={omega:.2f}, v={velocity:.2f}, frame {self.no_contour_count}/{self.max_no_contour_frames}")
        else:
            # If we've lost the line for too long, gradually reduce steering but maintain speed
            decay_factor = 0.8  # Reduce steering by 20% each frame after max_no_contour_frames
            omega = self.last_valid_omega * decay_factor
            velocity = self.last_valid_velocity
            
            # Update last valid omega for next iteration
            self.last_valid_omega = omega
            
            if DEBUG:
                self.loginfo(f"NO CONTOUR TOO LONG - Decaying omega: {omega:.2f}, v={velocity:.2f}")
                
        return omega, velocity


    def drive(self):
        '''Lane following'''
        if self.parking:
            return
        if self.disable_drive:
            if not self.manuvering:
                #stop at crosswalk
                self.twist.omega = 0
                self.twist.v = 0
                self.vel_pub.publish(self.twist)
            else:
                self.twist.v, self.twist.omega = self.manuver_around_bot()
                self.vel_pub.publish(self.twist)
        else:
            with self.lock:
                proportional = self.proportional
                multiple_points = list(self.multiple_points)  # make a copy to avoid shared access
                obj_stop = self.obj_stop
                if obj_stop:
                    self.obj_stop = False  # immediately clear the stop flag
                # duckie_stop = self.duckie_stop

            if obj_stop:
                self.stop(16)
                self.red_cooldown_until = time.time() + 8
                rospy.sleep(2.0)
                self.next_phase()
                return
            elif self.duckie_stop:
                self.twist.omega = 0
                self.twist.v = 0
                self.vel_pub.publish(self.twist)
                # self.stop(8)
                # rospy.sleep(1.0)
                # rospy.sleep(0.5)
                return


            if proportional is None:
                omega, velocity = self._handle_no_contour()
            else:
                self.no_contour_count = 0
                pid_output, P, D, I = self._calculate_pid(proportional)

                # Swap self.multiple_points for local copy
                self.multiple_points = multiple_points  # optional if downstream needs it
                omega, velocity, angle_adjustment = self._calculate_steering(pid_output, proportional)

                self.last_valid_omega = omega
                self.last_valid_velocity = velocity

                if DEBUG:
                    turn_dir = "RIGHT" if proportional > 0 else "LEFT" if proportional < 0 else "STRAIGHT"
                    debug_info = f"Turn: {turn_dir}, P:{P:.2f} D:{D:.2f} I:{I:.2f} Angle:{angle_adjustment:.2f} Omega:{omega:.2f} Speed:{velocity:.2f} Error:{proportional:.1f}"
                    self.loginfo(debug_info)

            self.twist.omega = omega
            self.twist.v = velocity
            self.vel_pub.publish(self.twist)
        
    def stop(self, duration):
        """Stop the robot for a specified duration"""
        self.twist.v = 0
        self.twist.omega = 0
        
        for i in range(duration):
            self.vel_pub.publish(self.twist)

    def hook(self):
        """Shutdown hook to safely stop the robot"""
        print("SHUTTING DOWN")
        self.twist.v = 0
        self.twist.omega = 0
        
        for i in range(8):
            self.vel_pub.publish(self.twist)

    def callback(self, msg):
        """Process camera image and detect lane"""
        img = self.jpeg.decode(msg.data)
        self.h, self.w = img.shape[:2]
        
        if self.stage == 1:
            if self.stage1_phases[self.phase] == 'tailing':
                # Dynamically adjust look-ahead distance
                if self.proportional is not None and abs(self.proportional) > self.large_error_threshold:
                    crop = img[350:-1, :, :]  # Look closer in turns
                else:
                    crop = img[300:-1, :, :]  # Standard distance
                    
                # Process image to find yellow lane
                proportional, multiple_points, red_detected, self.duckie_stop, _ = self._process_image(crop, img)

                with self.lock:
                    self.proportional = proportional
                    self.multiple_points = multiple_points
                    self.obj_stop = red_detected
            elif self.stage1_phases[self.phase] == 'right_turn':
                self.logwarn("performing right_turn")
                self.move(duration_sec=2.2, direction='right')
            elif self.stage1_phases[self.phase] == 'straight':
                self.logwarn("performing straight")
                self.move(duration_sec=3, direction='straight')
            elif self.stage1_phases[self.phase] == 'left_turn':
                self.logwarn("performing left_turn")
                if self.phase == 1:
                    self.move(duration_sec=2.8, direction='left', go_straight=0.9)
                else:
                    self.move(duration_sec=3.1, direction='left', go_straight=0.5)
            else:
                self.logwarn("done stage 1")

        elif self.stage == 2:
            cut_image = img[:, int(0.5 * self.w):]
            gray_image = cv2.cvtColor(cut_image, cv2.COLOR_BGR2GRAY)


            if not self.seen_tag:
                tags = self.at_detector.detect(gray_image, estimate_tag_pose=False, camera_params=None, tag_size=None)
                if len(tags) > 0:
                    self.react_to_detection(tags[0].tag_id)
                    self.last_tag_id = tags[0].tag_id
                    self.seen_tag = True
                    print(f"saw tag {tags[0].tag_id}")

            if self.stage2_phases[self.phase] == 'default':
                # Dynamically adjust look-ahead distance
                if self.proportional is not None and abs(self.proportional) > self.large_error_threshold:
                    crop = img[350:-1, :, :]  # Look closer in turns
                else:
                    crop = img[300:-1, :, :]  # Standard distance
                    
                # Process image to find yellow lane
                proportional, multiple_points, red_detected, self.duckie_stop, _ = self._process_image(crop)

                with self.lock:
                    self.proportional = proportional
                    self.multiple_points = multiple_points
                    self.obj_stop = red_detected
            elif self.stage2_phases[self.phase] == 'right':
                self.logwarn("performing right_turn")
                self.move(duration_sec=2.2, direction='right')
            elif self.stage2_phases[self.phase] == 'left':
                self.logwarn("performing left_turn")
                if self.last_tag_id == 48:
                    self.move(duration_sec=2.5, direction='left', go_straight=1.5)
                else:
                    self.move(duration_sec=3.0, direction='left', go_straight=0.8)

        elif self.stage == 3:
            cut_image = img[:, int(0.5 * self.w):]
            gray_image = cv2.cvtColor(cut_image, cv2.COLOR_BGR2GRAY)

            if self.stage3_phases[self.phase] == 'default':
                # Dynamically adjust look-ahead distance
                if self.proportional is not None and abs(self.proportional) > self.large_error_threshold:
                    crop = img[350:-1, :, :]  # Look closer in turns
                else:
                    crop = img[300:-1, :, :]  # Standard distance
                    
                # Process image to find yellow lane
                proportional, multiple_points, redline_stop, blueline_stop, blue_mask = self._process_image(crop)

                if blueline_stop:
                    #saw blue line
                    if self.blue_count == 1 or self.blue_count == 4:
                        if self.detect_peDuckstrians(img):
                            self.see_peduckstrian_timer = time.time()
                            self.disable_drive = True
                            print("SAW YELLOW DUCKS")
                        elif time.time() - self.see_peduckstrian_timer > 2:
                            self.blue_timer = time.time()
                            self.blue_count += 1
                            self.disable_drive = False
                            self.seen_blue = False

                    elif self.blue_count == 2 and not self.manuvering:

                        self.blue_timer = time.time()
                        self.manuvering = True

                else:
                    #lane follow
                    with self.lock:
                        self.proportional = proportional
                        self.multiple_points = multiple_points
                        self.obj_stop = redline_stop

            elif self.stage3_phases[self.phase] == 'end':
                self.move(duration_sec=0.2, direction='straight')

        elif self.stage == 4:
            if self.phase == 2:
                self.zone = (self.zone + 1)%5
                if self.zone == 0:
                    self.zone = 1
                self.reset_timer = time.time()
                print('NEW ZONE, GO GO GO GO')
                self.phase = 0
                
            else:
                if time.time() - self.reset_timer < 6:
                    return
                elif time.time() - self.reset_timer < 6.1:
                    self.disable_drive = False
                gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                if self.zone == 1 or self.zone == 4:
                    self.offset = -10
                    #cut right part

                    crop = img[300:-1, 430:-1, :]  # Standard distance
                else:
                    self.offset = 20
                    #cut left part

                    crop = img[300:-1, 0:210, :]  # Standard distance

                if self.stage4_phases[self.phase] == 'park':
                    self.park(zone=self.zone)
                    self.park_timer = time.time()
                elif self.stage4_phases[self.phase] == 'follow':
                    self.base_velocity = 0.17

                    if time.time() - self.tag_timer > 0.3:
                        # Check for AprilTags during parking follow phase
                        tags = self.at_detector.detect(gray_image, estimate_tag_pose=False, camera_params=None, tag_size=None)
                        
                        # If tags found, check size
                        if len(tags) > 0:
                            tag = tags[0]
                            # Calculate tag size (area of bounding box)
                            corners = tag.corners
                            x_min = min(corners[:, 0])
                            x_max = max(corners[:, 0])
                            y_min = min(corners[:, 1])
                            y_max = max(corners[:, 1])
                            
                            tag_width = x_max - x_min
                            tag_height = y_max - y_min
                            tag_area = tag_width * tag_height
                            
                            # print(f"AprilTag detected with area: {tag_area}")
                            
                            # If tag is large enough, we're close enough to stop
                            if tag_area > 20000:  # Adjust threshold as needed
                                print("Tag large enough, stopping. PARKED!!!!!!!!!!!!")
                                self.disable_drive = True
                                self.phase += 1
                                return
                        
                    if time.time() - self.park_timer > 10:
                        print("Timer hit. PARKED!!!!!!!!!!!!")
                        self.disable_drive = True
                        self.phase += 1
                    print('lane following now!!!')
                    proportional, multiple_points, redline_stop, blueline_stop, blue_mask = self._process_image(crop, img)
                    with self.lock:
                        self.proportional = proportional
                        self.multiple_points = multiple_points

        # Publish debug image if needed
        # rect_img_msg = CompressedImage(format="jpeg", data=self.jpeg.encode(crop))
        # self.pub.publish(rect_img_msg)

    def park(self, zone=1):
        """Perform a turning maneuver using angular velocity."""
        self.parking = True
        self.twist.v = 0.3
        print(f"turning to park for zone {zone}")
        start_time = rospy.get_time()

        """going straigh is -0.65, add to all angles"""
        # left is positive
        # right is negative
        self.twist.omega = 4
        self.twist.v = 0.4
        while rospy.get_time() - start_time < 0.2 and not rospy.is_shutdown():
            self.vel_pub.publish(self.twist)
            rospy.sleep(0.1)  # control frequency ~10Hz
        start_time = rospy.get_time()

        self.twist.v = 0.3
        if zone == 1:
            self.twist.omega = -2.9
            while rospy.get_time() - start_time < 2.5 and not rospy.is_shutdown():
                self.vel_pub.publish(self.twist)
                rospy.sleep(0.1)  # control frequency ~10Hz

        elif zone == 2:
            self.twist.omega = 0.3
            while rospy.get_time() - start_time < 0.7 and not rospy.is_shutdown():
                self.vel_pub.publish(self.twist)
                rospy.sleep(0.1)  # control frequency ~10Hz
            start_time = rospy.get_time()

            self.twist.omega = -3.0
            while rospy.get_time() - start_time < 2.7 and not rospy.is_shutdown():
                self.vel_pub.publish(self.twist)
                rospy.sleep(0.1)  # control frequency ~10Hz

        elif zone == 3:

            #old zone 3
            # self.twist.omega = 3.2
            # while rospy.get_time() - start_time < 2.7 and not rospy.is_shutdown():
            #     self.vel_pub.publish(self.twist)
            #     rospy.sleep(0.1)  # control frequency ~10Hz
            # start_time = rospy.get_time()

            # self.twist.omega = -0.65
            # while rospy.get_time() - start_time < 1 and not rospy.is_shutdown():
            #     self.vel_pub.publish(self.twist)
            #     rospy.sleep(0.1)  # control frequency ~10Hz

            self.twist.omega = 4
            while rospy.get_time() - start_time < 2 and not rospy.is_shutdown():
                self.vel_pub.publish(self.twist)
                rospy.sleep(0.1)  # control frequency ~10Hz
            start_time = rospy.get_time()

            self.twist.omega = -0.65
            while rospy.get_time() - start_time < 1 and not rospy.is_shutdown():
                self.vel_pub.publish(self.twist)
                rospy.sleep(0.1)  # control frequency ~10Hz
            
            
        elif zone == 4:

            #old zone 4
            # self.twist.omega = 3.6
            # while rospy.get_time() - start_time < 1.8 and not rospy.is_shutdown():
            #     self.vel_pub.publish(self.twist)
            #     rospy.sleep(0.1)  # control frequency ~10Hz
            # start_time = rospy.get_time()

            # self.twist.omega = 2
            # while rospy.get_time() - start_time < 0.5 and not rospy.is_shutdown():
            #     self.vel_pub.publish(self.twist)
            #     rospy.sleep(0.1)  # control frequency ~10Hz
            # start_time = rospy.get_time()

            # self.twist.omega = -0.65
            # while rospy.get_time() - start_time < 1 and not rospy.is_shutdown():
            #     self.vel_pub.publish(self.twist)
            #     rospy.sleep(0.1)  # control frequency ~10Hz

            self.twist.omega = 3.2
            while rospy.get_time() - start_time < 2.7 and not rospy.is_shutdown():
                self.vel_pub.publish(self.twist)
                rospy.sleep(0.1)  # control frequency ~10Hz
            start_time = rospy.get_time()

            self.twist.omega = -0.65
            while rospy.get_time() - start_time < 1 and not rospy.is_shutdown():
                self.vel_pub.publish(self.twist)
                rospy.sleep(0.1)  # control frequency ~10Hz
            

        # Stop briefly after turn
        self.stop(4)
        self.parking = False
        self.phase += 1
        return True  # signal that turn is complete
    
    def react_to_detection(self, tag_id):
        if tag_id == 48:
            self.stage2_phases = ['default', 'left', 'right'] # turning left
        # else:
        #     self.stage2_phases = ['default', 'right', 'left']  # turning right

    def move(self, duration_sec=1.5, direction='right', go_straight=0):
        """Perform a turning maneuver using angular velocity."""
        turn_twist = Twist2DStamped()
        print(f"direction changed {direction}")

        # Set base turning parameters
        if direction == 'right':
            turn_twist.v = self.base_velocity * 1.51
            turn_twist.omega = -4.5  # right turn (omega < 0)
        elif direction == 'left':
            turn_twist.v = self.base_velocity * 1.63
            turn_twist.omega = -0.5 # straight
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < go_straight and not rospy.is_shutdown():
                self.vel_pub.publish(turn_twist)
                rospy.sleep(0.1)
            turn_twist.v = self.base_velocity * 1.5
            turn_twist.omega = 3.7  # left turn (omega > 0)
        else:
            turn_twist.v = self.base_velocity * 1.63
            turn_twist.omega = -0.5 # straight

        # Get current time and spin for a fixed duration
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < duration_sec and not rospy.is_shutdown():
            self.vel_pub.publish(turn_twist)
            rospy.sleep(0.1)  # control frequency ~10Hz

        # Stop briefly after turn
        self.stop(4)
        self.phase = 0
        print(f"done with {direction}")
        if self.stage == 1 and self.max_phase == 3:
            self.next_stage()
            self.publish_leds([0, 0, 0])

        if self.stage == 2 and self.max_phase == 2:
            self.next_stage()

        if self.stage == 3 and self.max_phase == 1:
            self.next_stage()

        return True  # signal that turn is complete

    def next_phase(self):
        if self.stage == 1 and self.max_phase == 0:
            # Decide turn direction using stored cx history
            turn_decision = self.detector.get_direction()  # returns "LEFT", "RIGHT", or None
            print('decision made')
            if turn_decision == "LEFT":
                self.stage1_phases = ['tailing', 'left_turn', 'straight', 'right_turn']
        self.max_phase += 1
        self.phase = self.max_phase

    def next_stage(self):
        self.stage += 1
        print(f"entered new stage {self.stage}")
        self.phase = 0
        self.max_phase = 0
        if self.stage == 3:
            self.blue_timer = time.time()

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

    def manuver_around_bot(self):
        self.state_time += 1
        turn_angle = 5
        if self.state_time < 5:
            return 0, 0

        # wait for 1 second
        if self.manuver_state == 0:
            print("start maneuvering...")
            if self.state_time > 8:
                self.manuver_state += 1
                self.state_time = 0
            vel = 0
            twist = 0
        # turn to face other lane, left
        elif self.manuver_state == 1:
            if self.state_time > 9:
                self.manuver_state += 1
                self.state_time = 0
            vel = -0.25
            twist = turn_angle
        # drive into other lane 
        elif self.manuver_state == 2:
            if self.state_time > 12:
                self.manuver_state += 1
                self.state_time = 0
            vel = 0.25
            twist = 0
        # turn before drive past other bot, right
        elif self.manuver_state == 3:
            if self.state_time > 8:
                self.manuver_state += 1
                self.state_time = 0
            vel = 0.25
            twist = -turn_angle
        # drive past other bot
        elif self.manuver_state == 4:
            if self.state_time > 34:
                self.manuver_state += 1
                self.state_time = 0
            vel = 0.25
            twist = -1.9
        # turn to lane follow again, left
        elif self.manuver_state == 5:
            if self.state_time > 8:
                self.manuver_state += 1
                self.state_time = 0
            vel = 0.25
            twist = turn_angle
        # end manuver
        else:
            self.manuvering = False
            self.seen_blue = False
            self.disable_drive = False
            self.blue_count = 3
            self.state_time = 0
            self.manuver_state = 0
            vel = 0.2
            twist = 0
        
        return vel, twist


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Lane following node')
    parser.add_argument('--zone', type=int, help='zone for parking')

    args = parser.parse_args()

    print(f"Zone argument received: {args.zone}")

    node = LaneFollowNode("lanefollow_node", zone=args.zone)
    rate = rospy.Rate(8)  # 8hz
    while not rospy.is_shutdown():
        node.drive()
        rate.sleep()