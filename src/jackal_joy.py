#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class RosNode:
    def __init__(self):
        rospy.init_node("JackalJoyNode")
        
        # Get launch file params
        self.linear_speed = rospy.get_param("~linear_speed")
        self.angular_speed = rospy.get_param("~angular_speed")
        # Private variables
        self._has_new_input = False
        # Set Class Params
        self.sub_joy = rospy.Subscriber("ps3/joy",
                                        Joy,
                                        self.callback,
                                        queue_size=10)
        self.pub_cmd_vel = rospy.Publisher("button/cmd_vel", Twist, queue_size=10)
        rospy.spin()
    
    def callback(self, msg):
        self.button_control(msg)
        
    def button_control(self, msg):
        cmd_vel = Twist()
        up    = msg.buttons[0]
        down  = msg.buttons[1]
        left  = msg.buttons[2]
        right = msg.buttons[3]
        uldr  = [up,down,left,right]
        
        if hasTrue(uldr):
            cmd_vel.linear.x = self.linear_speed * (up - down)
            cmd_vel.angular.z = self.angular_speed * (left - right)
            self._has_new_input = True
            self.pub_cmd_vel.publish(cmd_vel)
        elif self._has_new_input:
            self._has_new_input = False
            self.pub_cmd_vel.publish(cmd_vel)
        else:
            pass
        
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
