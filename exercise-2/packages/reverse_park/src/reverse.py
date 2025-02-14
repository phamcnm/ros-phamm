#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, LEDPattern
from std_msgs.msg import String, ColorRGBA
from std_msgs.msg import Header


import time

STATES = {
    '0': [0, 0, 0],
    '1': [1, 0, 1],
    '2': [0, 1, 0],
    '3': [0, 0, 1]
}

# throttle and direction for each wheel
THROTTLE_LEFT = 0.66        # 50% throttle
DIRECTION_LEFT = 1         # forward
THROTTLE_RIGHT = 0.7       # 30% throttle
DIRECTION_RIGHT = 1       # backward


class Driver(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(Driver, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        signal_topic = f"/{self._vehicle_name}/led_emitter_node/led_pattern"
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
        
        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

        # for LED
        self._publisher_led = rospy.Publisher(signal_topic, LEDPattern, queue_size=1)
        self.state = '1'
        self.needs_light_update = True

        self.start_time = None

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
    
    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(20)
        phase = -0.5
        starting = None

        while not rospy.is_shutdown(): # change this into 1.25 memters
            if self.start_time is None:
                self.start_time = time.time()
            if time.time() - self.start_time < 1: # wait 3 seconds until moving
                self.publish_light(rate, '1')
                continue

            # self._publisher_signal.publish('2')

            if self._ticks_right is None or self._ticks_left is None or self._starting_ticks_left is None:
                continue

            if not starting:
                starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            
            if phase == -1:
                self.needs_light_update == True
                # self.state == '2'
                phase = -0.5

            if phase == -0.5:
                self.publish_light(rate, '2')
                phase = 0

            if phase == 0 and not self.drive_straight(rate, starting, 0.5): # long sided D straight
                phase = 0.5
                starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
                t = time.time()

            if phase == 0.5 and not self.wait(rate, t, 1):
                phase = 1
                starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)

            if phase == 1 and not self.rotate(rate, starting, 0.048, direction='left'): # first turn
                phase = 1.5
                starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
                t = time.time()

            if phase == 1.5 and not self.wait(rate, t, 1):
                phase = 2
                starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)

            if phase == 2 and not self.drive_reverse(rate, starting, 0.3): # reverse
                phase = 2.5
                starting = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
                t = time.time()

            if phase == 2.5:
                self.publish_light(rate, '3')
                self.on_shutdown()
                rospy.signal_shutdown("Finished")

    def drive_straight(self, rate, starting, distance):
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
        if (cur_pos - starting) / 135 < distance:
            cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self._publisher.publish(message)
            rate.sleep()
        else:
            return False
        return True
    
    def drive_reverse(self, rate, starting, distance):
        message = WheelsCmdStamped(vel_left=-self._vel_left, vel_right=-self._vel_right)
        cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
        if (cur_pos - starting) / 135 > -distance:
            cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self._publisher.publish(message)
            rate.sleep()
        else:
            return False
        return True

    def rotate(self, rate, starting, degree, direction='right'):
        cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
        if direction == 'right':
            message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=-self._vel_right)
        else:
            message = WheelsCmdStamped(vel_left=-self._vel_left, vel_right=self._vel_right)
        if (cur_pos-starting) / 135 > -degree:
            cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            self._publisher.publish(message)
            rate.sleep()
        else:
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
    
    def publish_light(self, rate, state):
        colors = STATES[state]
        rgba_val = ColorRGBA(r=colors[0], g=colors[1], b=colors[2], a=1)
        self._publisher_led.publish(LEDPattern(rgb_vals=[rgba_val,rgba_val,rgba_val,rgba_val,rgba_val]))
        rate.sleep()
        self.needs_light_update = False

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        # rgba_val = ColorRGBA(r=0, g=0, b=0, a=1)
        # self._publisher_led.publish(LEDPattern(rgb_vals=[rgba_val,rgba_val,rgba_val,rgba_val,rgba_val]))
        self._publisher.publish(stop)



if __name__ == '__main__':
    # create the node
    node = Driver(node_name='wheel_encoder_reader_node')
    # run the timer in node
    node.run()
    # keep spinning
    rospy.spin()
