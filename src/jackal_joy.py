#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class RosNode:
    def __init__(self):
        rospy.init_node("JackalJoyNode")
        
        # Get launch file params
        self.linear_speed =  rospy.get_param("~linear_speed")
        self.angular_speed = rospy.get_param("~angular_speed")
        self.joy_topic =     rospy.get_param("~joy_topic")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic")
        # Private variables
        self._has_new_input = False
        # Set Class Params
        self.sub_joy = rospy.Subscriber(self.joy_topic,
                                        Joy,
                                        self.callback,
                                        queue_size=10)
        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.pub_start_stop = rospy.Publisher("start_stop", Bool, queue_size=10)
        rospy.spin()
    
    def callback(self, msg):
        self.dpad_control(msg)
        self.start_stop(msg)
        
    def dpad_control(self, msg):
        cmd_vel = Twist()
        up    = msg.buttons[4]
        left  = msg.buttons[5]
        down  = msg.buttons[6]
        right = msg.buttons[7]
        uldr  = [up,left,down,right]
        
        if hasTrue(uldr):
            cmd_vel.linear.x = self.linear_speed * (up - down)
            cmd_vel.angular.z = self.angular_speed * (right - left)
            self._has_new_input = True
            self.pub_cmd_vel.publish(cmd_vel)
        elif self._has_new_input:
            self._has_new_input = False
            self.pub_cmd_vel.publish(cmd_vel)
        else:
            pass
        
    def start_stop(self, msg):
        start_stop = Bool()
	t = msg.buttons[12]
        o = msg.buttons[13]
        x = msg.buttons[14]
	s = msg.buttons[15]
        if x == True:
	    start_stop.data = True
            self.pub_start_stop.publish(start_stop)
        elif o == True:
            start_stop.data = False
            self.pub_start_stop.publish(start_stop)
        
def hasTrue(mylist):
    for item in mylist:
        if item == True:
            return True
    return False
    
if __name__ == "__main__":
    try:
        rosnode = RosNode()
    except rospy.ROSInterruptException:
        pass
