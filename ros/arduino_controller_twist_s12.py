#!/usr/bin/env python
##############################################################################
# MobileBase controller for MockBot2
# This is a simple control of the 2 servos connected to the Arduino UNO board
# Author: chrimo@moccy.xdsnet.de
#
# Todo: calibration of mockbot2 motors and motor controller
# Todo:
#
##############################################################################

import roslib
import rospy

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import *

left_speed_out=0
right_speed_out=0

def cmd_vel_callback(cmd_vel):
    global left_speed_out,right_speed_out
    ROBOT_WIDTH=0.125
    left_speed_out = cmd_vel.linear.x*150 - cmd_vel.angular.z*ROBOT_WIDTH/2*150
    right_speed_out = cmd_vel.linear.x*150 + cmd_vel.angular.z*ROBOT_WIDTH/2*150
    v = cmd_vel.linear.x        # speed m/s
    theta = cmd_vel.angular.z      # angle rad/s
    rospy.loginfo("VEL_CMD: " + str(v) + "," + str(theta))
    motor_publisher(left_speed_out, right_speed_out)
def vel_cmd_listener():
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
def motor_publisher(left,right):
    lwheel_pub= rospy.Publisher('/left_motor', Int16, queue_size=10)
    rwheel_pub= rospy.Publisher('/right_motor', Int16, queue_size=10)
    rospy.loginfo(left)
    rospy.loginfo(right)
    lwheel_pub.publish(left)
    rwheel_pub.publish(right)



if __name__ == '__main__':
    try:
        rospy.init_node('BaseController')
        vel_cmd_listener()
        rospy.loginfo(left_speed_out)
        rospy.loginfo(right_speed_out)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


