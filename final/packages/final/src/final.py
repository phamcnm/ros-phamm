#!/usr/bin/env python3

import rospy
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

class Controller:
    def __init__(self, target_span=95.0, min_span=80.0, max_span=130.0, history_len=10):
        self.target_span = target_span
        self.min_span = min_span
        self.max_span = max_span
        self.spans = deque(maxlen=history_len)
        self.middles = deque(maxlen=history_len)

    def update(self, new_span, new_middle):
        """Add new horizontal span (can be None if undetected)"""
        self.spans.append(new_span)
        self.middles.append(new_middle) # stores the point centers[1][4] in the middle of grid

    def compute_base_velocity(self, phase):
        """Compute velocity based on average of recent spans
            phase 0 is 'tailing', phase 1 is 'laning'
        """
        valid_spans = [s for s in self.spans if s is not None]
        if phase == 1 and len(valid_spans) == 0:
            return 0.25  # not detecting blue (ie, in laning) and no valid data
        elif len(valid_spans) == 0:
            return 0.15 # no data, but detected blue, so slow down
        
        avg_span = np.mean(valid_spans)

        if avg_span >= self.max_span:
            return 0.0  # too close → stop
        elif avg_span <= self.min_span:
            return 0.23  # too far → speed up
        else:
            # Linearly interpolate between span=78→130 and velocity=0.25→0.0
            # You can tune the values as needed
            velocity = np.interp(avg_span,
                                 [self.min_span, self.target_span, self.max_span],
                                 [0.23, 0.2, 0.0])
            return velocity

    def compute_direction(self):
        # takes in x coordinates of points, so self.middles are list of integers
        valid_x = [pt for pt in self.middles if pt is not None and pt > 0]

        if len(valid_x) < 2:
            return None

        first = valid_x[0]
        last = valid_x[-1]
        return "right" if last - first > 0 else "left"
    
class LaneFollowNode(DTROS):
    def __init__(self, node_name):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = os.environ['VEHICLE_NAME']
        
        # Initialize TurboJPEG for image decoding
        self.jpeg = TurboJPEG()
        
        # Set up publishers and subscribers
        self._setup_publishers_subscribers()
        
        # Initialize control parameters
        self._init_control_params()

        self._init_logic_params()

        self.last_stamp = rospy.Time.now()

        self.cbParametersChanged()

        # self.grid_controller = Controller()

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
        rgba_val = ColorRGBA(r=colors[0], g=colors[1], b=colors[2], a=1)
        self._publisher_led.publish(LEDPattern(rgb_vals=[rgba_val,rgba_val,rgba_val,rgba_val,rgba_val]))

    def detect_peDuckstrians(self, hsvFrame):
        self.orange_lower = np.array([10, 120, 120], np.uint8)
        self.orange_upper = np.array([20, 255, 255], np.uint8)

        orange_mask = cv2.inRange(hsvFrame, self.orange_lower, self.orange_upper)
        
        contours, hierarchy = cv2.findContours(orange_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)

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
            self.offset = 220
            
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
        self.bot_detected = False
        self.manuvering = False
        self.manuver_state = 0
        self.state_time = 0

    def _init_logic_params(self):
        self.phase = 0
        self.red_cooldown_until = 0
        self.max_phase = 0
        self.stage1_phases = ['tailing', 'right_turn', 'straight', 'left_turn']

        #stage 2 stuff
        self.stage = 1

        self.stage2_phases = ['default', 'right', 'left']
        self.seen_tag = False
        self.driving = True

        #stage 3 stuff
        self.stage3_phases = ['default', 'end']

    def cb_tof(self, msg):
        """Process Time-of-Flight sensor data"""
        self.tof_distance = msg.range
        if 0.05 < self.tof_distance <= 0.3:
            self.obj_stop = True

    def _process_image(self, crop):
        """Process image to find lane and calculate proportional error"""
        crop_width = crop.shape[1]
        crop_height = crop.shape[0]
        
        # Convert to HSV and create mask for yellow lane
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])

        if self.stage == 1 or self.stage == 2 or self.stage == 3:
            if time.time() > self.red_cooldown_until:
                red_mask = cv2.inRange(hsv, RED_MASK[0], RED_MASK[1])
                red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                    
                for i, contour in enumerate(red_contours):
                    for point in contour:
                        x, y = point[0]
                        if y >= 0.7 * crop_height:
                            print("red detected") 
                            return None, [], True, False
                    
        now = rospy.Time.now()
        if self.stage == 1 and self.phase == 0 and now - self.last_stamp > self.publish_duration:
            self.last_stamp = now
            # hsv_full = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # blue_mask = cv2.inRange(crop, BLUE_MASK[0], BLUE_MASK[1])
            # blue_count = cv2.countNonZero(blue_mask)
            # print(blue_count)
            # blue_mask = cv2.inRange(hsv, BLUE_MASK[0], BLUE_MASK[1])
            # blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            # large_blue_contours = [cnt for cnt in blue_contours if cv2.contourArea(cnt) > 40]
            # num_large_blue = len(large_blue_contours)
            
            # if num_large_blue > 0:
            #     # self.phase = 0
            #     print('seen blue', num_large_blue)
            #     return None, [], False, True

            # DETECT CIRCULAR GRID
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            equalized = clahe.apply(gray)
            _, thresh = cv2.threshold(equalized, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            (detection, centers) = cv2.findCirclesGrid(thresh, patternSize=(7, 3), 
                flags=cv2.CALIB_CB_SYMMETRIC_GRID, blobDetector=self.simple_blob_detector)
            if detection or (centers is not None and len(centers) >= 3):
                return None, [], False, True
            # if centers is not None and len(centers) == 21:
            #     centers = centers.reshape((3, 7, 2))
            #     middle_point = centers[1][4]

            #     left = centers[1, 0]
            #     right = centers[1, -1]
            #     horizontal_span = np.linalg.norm(right - left)
            #     self.grid_controller.update(horizontal_span, middle_point[0])
            # else:
            #     self.grid_controller.update(None, None)
   
        if self.stage == 3:
            bot_detected = self.detect_bot(crop)
            if bot_detected:
                self.manuvering = True

            if not bot_detected and not self.manuvering:
                self.blue_lower = np.array([100, 150, 50], np.uint8)  
                self.blue_upper = np.array([115, 255, 255], np.uint8)

                blueline_mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)

                blueline_contours, _ = cv2.findContours(blueline_mask, 
                                                cv2.RETR_TREE, 
                                                cv2.CHAIN_APPROX_SIMPLE)
                if self.seen_line == 2:
                    if len(blueline_contours) == 0:
                        self.seen_line = 0
                    else:
                        return None, [], False, False

                for point in blueline_contours:
                    x, y = point[0][0]
                    if y >= 0.45 * self.h:
                        print("GOT IN BLUE LINE")
                        self.seen_line = 1
                        self.blueline_start_time = time.time()
                        self.blueline_stop = True
                        break

                
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
        
        return proportional, multiple_points, False, False
    
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
        if self.disable_drive:
            if not self.bot_detected and not self.manuvering:
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
                duckie_stop = self.duckie_stop
                if duckie_stop:
                    self.duckie_stop = False

        if obj_stop:
            self.stop(8)
            self.red_cooldown_until = time.time() + 10
            rospy.sleep(1.0)
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
                proportional, multiple_points, red_detected, self.duckie_stop = self._process_image(crop)

                with self.lock:
                    self.proportional = proportional
                    self.multiple_points = multiple_points
                    self.obj_stop = red_detected
            elif self.stage1_phases[self.phase] == 'right_turn':
                self.logwarn("performing right_turn")
                self.move(duration_sec=2.2, direction='right')
            elif self.stage1_phases[self.phase] == 'straight':
                self.logwarn("performing straight")
                self.move(duration_sec=3.69, direction='straight')
            elif self.stage1_phases[self.phase] == 'left_turn':
                self.logwarn("performing left_turn")
                self.move(duration_sec=3, direction='left')
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

            print(f"self.phase {self.phase}")

            if self.stage2_phases[self.phase] == 'default':
                # Dynamically adjust look-ahead distance
                if self.proportional is not None and abs(self.proportional) > self.large_error_threshold:
                    crop = img[350:-1, :, :]  # Look closer in turns
                else:
                    crop = img[300:-1, :, :]  # Standard distance
                    
                # Process image to find yellow lane
                proportional, multiple_points, red_detected, self.duckie_stop = self._process_image(crop)

                with self.lock:
                    self.proportional = proportional
                    self.multiple_points = multiple_points
                    self.obj_stop = red_detected
            elif self.stage2_phases[self.phase] == 'right':
                self.logwarn("performing right_turn")
                self.move(duration_sec=2.2, direction='right')
            elif self.stage2_phases[self.phase] == 'left':
                self.logwarn("performing left_turn")
                self.move(duration_sec=2.6, direction='left')

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
                proportional, multiple_points, blueline_detected, _ = self._process_image(img)

                if self.blueline_stop:
                    #saw blue line
                    print('STOPPING RIGHT NOW')
                    if self.seen_line == 1:
                        if time.time() - self.blueline_start_time < 1:
                            self.disable_drive = True
                        else:
                            self.seen_line = 2
                            self.disable_drive = False
                    if self.seen_line == 2:
                        hsvFrame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                        if self.detect_peDuckstrians(hsvFrame):
                            print("SAW YELLOW DUCKS")
                            self.disable_drive = True
                        else:
                            self.blueline_stop = False
                            self.disable_drive = False
                else:
                    #lane follow

                    with self.lock:
                        self.proportional = proportional
                        self.multiple_points = multiple_points
                        self.obj_stop = blueline_detected

            elif self.stage3_phases[self.phase] == 'end':
                return
            
        # Publish debug image if needed
        if DEBUG:
            crop = img[300:-1, :, :]
            rect_img_msg = CompressedImage(format="jpeg", data=self.jpeg.encode(crop))
            self.pub.publish(rect_img_msg)
    
    def react_to_detection(self, tag_id):
        if tag_id == 48:
            self.stage2_phases = ['default', 'left', 'right'] # turning left
        # else:
        #     self.stage2_phases = ['default', 'right', 'left']  # turning right

    def move(self, duration_sec=1.5, direction='right'):
        """Perform a turning maneuver using angular velocity."""
        turn_twist = Twist2DStamped()
        print(f"direction {direction}")

        # Set base turning parameters
        if direction == 'right':
            turn_twist.v = self.base_velocity * 1.51
            turn_twist.omega = -4.5  # right turn (omega < 0)
        elif direction == 'left':
            turn_twist.v = self.base_velocity * 1.45
            turn_twist.omega = 3.5  # left turn (omega > 0)
        else:
            turn_twist.v = self.base_velocity * 1.4
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

        if self.stage == 2 and self.max_phase == 2:
            self.next_stage()

        return True  # signal that turn is complete

    def next_phase(self):
        self.max_phase += 1
        self.phase = self.max_phase

    def next_stage(self):
        print(f"entered new stage {self.stage}")
        self.stage += 1
        self.phase = 0
        self.max_phase = 0

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

    def detect_bot(self, image_cv):
        """
        Callback for processing a image which potentially contains a back pattern. Processes the image only if
        sufficient time has passed since processing the previous image (relative to the chosen processing frequency).

        The pattern detection is performed using OpenCV's `findCirclesGrid <https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?highlight=solvepnp#findcirclesgrid>`_ function.

        Args:
            image_msg (:obj:`sensor_msgs.msg.CompressedImage`): Input image

        """
        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now

        (detection, centers) = cv2.findCirclesGrid(
            image_cv,
            patternSize=tuple(self.circlepattern_dims),
            flags=cv2.CALIB_CB_SYMMETRIC_GRID,
            blobDetector=self.simple_blob_detector,
        )

        # if the detection is successful add the information about it,
        # otherwise publish a message saying that it was unsuccessful
        if detection > 0:
            points_list = []
            for point in centers:
                center = Point32()
                center.x = point[0, 0]
                center.y = point[0, 1]
                center.z = 0
                points_list.append(center)
        
        if detection > 0:
            return np.max(centers) - np.min(centers) > 250
        return False

    def manuver_around_bot(self):
        print("MANUEVERING...")
        self.state_time += 1
        turn_angle = 10
        turn_time = 12
        straight_time = 50
        if self.state_time < 5:
            return 0, 0

        # wait for 1 second
        if self.manuver_state == 0:
            if self.state_time > 25:
                self.manuver_state += 1
                self.state_time = 0
            vel = 0
            twist = 0
        # turn to face other lane
        elif self.manuver_state == 1:
            if self.state_time > turn_time:
                self.manuver_state += 1
                self.state_time = 0
            vel = -0.25
            twist = turn_angle
        # drive into other lane 
        elif self.manuver_state == 2:
            if self.state_time > straight_time:
                self.manuver_state += 1
                self.state_time = 0
            vel = 0.25
            twist = 0
        # turn to drive past other bot
        elif self.manuver_state == 3:
            if self.state_time > turn_time:
                self.manuver_state += 1
                self.state_time = 0
            vel = 0.25
            twist = -turn_angle
        # drive past other bot
        elif self.manuver_state == 4:
            if self.state_time > straight_time:
                self.manuver_state += 1
                self.state_time = 0
            vel = 0.25
            twist = 0
        # turn back to face proper lane
        elif self.manuver_state == 5:
            if self.state_time > turn_time:
                self.manuver_state += 1
                self.state_time = 0
            vel = 0.25
            twist = -turn_angle
        # drive into proper lane
        elif self.manuver_state == 6:
            if self.state_time > straight_time:
                self.manuver_state += 1
                self.state_time = 0
            vel = 0.25
            twist = 0
        # turn to face proper direction
        elif self.manuver_state == 7:
            if self.state_time > turn_time:
                self.manuver_state += 1
                self.state_time = 0
            vel = 0.25
            twist = turn_angle
        # end manuver
        else:
            self.manuvering = False
            self.state_time = 0
            self.manuver_state = 0
            vel = 0.2
            twist = 0
        
        return vel, twist


if __name__ == "__main__":
    node = LaneFollowNode("lanefollow_node")
    rate = rospy.Rate(8)  # 8hz
    while not rospy.is_shutdown():
        node.drive()
        rate.sleep()