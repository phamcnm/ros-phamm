#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped

# throttle and direction for each wheel
THROTTLE_LEFT = 0.64        # 50% throttle
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
        self._starting_ticks_right = None
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
        if not self._starting_ticks_right:
            self._starting_ticks_right = self._ticks_right
    
    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(100)
        print(self._radius)
        reached = False
        backed = False

        counter = 0

        while not rospy.is_shutdown(): # change this into 1.25 memters
            if self._ticks_right is not None and self._ticks_left is not None and self._starting_ticks_left is not None:
                cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
                # message = WheelsCmdStamped(vel_left= -self._vel_left, vel_right=self._vel_right)
                # self._publisher.publish(message)
                rate.sleep()
                
                
                # print(f"Wheel encoder ticks [LEFT, RIGHT]: {self._ticks_left}, {self._ticks_right}")
                # cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
                # message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)

                if reached is False and cur_pos / 135 < 0.055:
                    message = WheelsCmdStamped(vel_left= self._vel_left, vel_right=-self._vel_right)
                    cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
                    self._publisher.publish(message)
                    rate.sleep()
                else:
                    stop = WheelsCmdStamped(vel_left=0, vel_right=0)
                    self._publisher.publish(stop)
                    rate.sleep()
                    reached = True

                if reached and counter < 100:
                    stop = WheelsCmdStamped(vel_left=0, vel_right=0)
                    self._publisher.publish(stop)
                    rate.sleep()
                    counter += 1
                
                if counter < 100:
                    continue

                if reached and not backed:
                    print('cur_pos ', cur_pos)
                    message = WheelsCmdStamped(vel_left=-self._vel_left, vel_right=self._vel_right)
                    if cur_pos / 135 >= 0:
                        cur_pos = 2 * 3.14159 * self._radius * (self._ticks_left - self._starting_ticks_left)
                        self._publisher.publish(message)
                        rate.sleep()
                    else:
                        backed = True

                if reached and backed:
                    message = WheelsCmdStamped(vel_left=0, vel_right=0)
                    self._publisher.publish(message)
                    rate.sleep()
                    rospy.signal_shutdown("Finished")

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
