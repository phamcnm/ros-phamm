#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
import time

# throttle and direction for each wheel
THROTTLE_LEFT = 0.65        # 50% throttle
DIRECTION_LEFT = 1         # forward
THROTTLE_RIGHT = 0.7       # 30% throttle
DIRECTION_RIGHT = 1       # backward

class WheelEncoderReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelEncoderReaderNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        # temporary data storage
        self._ticks_left = None
        self._ticks_right = None
        self._starting_ticks_left = None
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT 
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        self._radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 100)
        
        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

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
    
    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        start_time = None

        phase = 0
        while not rospy.is_shutdown(): # change this into 1.25 memters
            if self._ticks_right is None or self._ticks_left is None or self._starting_ticks_left is None:
                continue
            cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)

            if phase == 0:
                if cur_pos / 135 < 1.25:
                    cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
                    self._publisher.publish(message)
                    rate.sleep()
                    continue
                else:
                    phase = 0.5
                    start_time = time.time()

            if phase == 0.5:
                if time.time() - start_time < 1:
                    self.on_shutdown()
                    rate.sleep()
                    continue
                else:
                    phase = 1

            if phase == 1:
                message = WheelsCmdStamped(vel_left=self._vel_left * -1, vel_right=self._vel_right * -1)
                if cur_pos / 135 >= 0:
                    cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
                    self._publisher.publish(message)
                    rate.sleep()
                    continue
                else:
                    phase = 1.5

            if phase == 1.5:
                self.on_shutdown()
                rate.sleep()
                rospy.signal_shutdown("Finished")
            
            # if reached is False and cur_pos / 135 < 1.25:
            #     print(cur_pos)
            #     cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            #     self._publisher.publish(message)
            #     rate.sleep()
            #     continue
            # else:
            #     reached = True
            #     start_time = time.time()
            
            # if start_time is None or time.time() - start_time < 1:
            #     self.on_shutdown()
            #     rate.sleep()
            #     continue
        
            # if reached and not backed:
            #     message = WheelsCmdStamped(vel_left=self._vel_left * -1, vel_right=self._vel_right * -1)
            #     if cur_pos / 135 >= 0:
            #         cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
            #         self._publisher.publish(message)
            #         rate.sleep()
            #     else:
            #         backed = True

            # if reached and backed:
            #     message = WheelsCmdStamped(vel_left=0, vel_right=0)
            #     self._publisher.publish(message)
            #     rate.sleep()

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
    # run the timer in node
    node.run()
    # keep spinning
    rospy.spin()
