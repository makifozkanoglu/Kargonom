#!/usr/bin/env python
##############################################################################
# Base controller for MockBotBrickPi
# This is a simple control of the 4 servos connected to the BrickPi board
# Author: chrimo@moccy.xdsnet.de
#
##############################################################################

import roslib
import rospy
import time

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

#Model dependend settings
PI=3.141
ROBOT_WIDTH=0.125
WHEEL_DIAMETER=0.065
WHEEL_RADIUS=WHEEL_DIAMETER/2
WHEEL_PERIMETER=2*PI*WHEEL_RADIUS

#RPM dependant from voltage without load
#9.0V = 120 RPM
#7.5V = 80 RPM
# with load 60-80 rpm is a good average
MAX_RPM=250.0
RPS=MAX_RPM/60.0
left_whell=Int16()
left_whell.data=0
right_whell=Int16()
right_whell.data=0

MPS=RPS*WHEEL_PERIMETER

PWRDIV=1000*RPS

rospy.loginfo("PWRDIV:"+str(PWRDIV))

class MotorController(object):
    global left_speed_out
    global right_speed_out
    def __init__(self):
        """
        :rtype: object
        """
        self.vel_cmd_listener()
        self.publ=rospy.Publisher('/left_motor', Int16, queue_size=10)
        self.pubr=rospy.Publisher('/right_motor', Int16, queue_size=10)
        self.motorvalue_publish()
    def cmd_vel_callback(self,cmd_vel):
        left_speed_out = cmd_vel.linear.x - cmd_vel.angular.z*ROBOT_WIDTH/2
        right_speed_out = cmd_vel.linear.x + cmd_vel.angular.z*ROBOT_WIDTH/2
        v = cmd_vel.linear.x        # speed m/s
        theta = cmd_vel.angular.z      # angle rad/s
        rospy.loginfo("VEL_CMD_CB: v:" + str(v) + ", theta:" + str(theta))
        self.motor_control(left_speed_out, right_speed_out)
    def vel_cmd_listener(self):
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
    def motor_control(self,left_speed_out,right_speed_out):
        rospy.loginfo("LSPEED:"+ str(left_speed_out) + " RSPEED:" + str(right_speed_out))
        rospy.loginfo("LSPEED:"+ str(left_speed_out) + " RSPEED:" + str(right_speed_out))
        right_whell.data = int(-right_speed_out*PWRDIV)
        rospy.loginfo("RF:"+str(right_whell.data))
        left_whell.data = int(-left_speed_out*PWRDIV)
        rospy.loginfo("LF:"+str(left_whell.data))
    def motorvalue_publish(self):
        self.publ.publish(left_whell)
        self.pubr.publish(right_whell)

if __name__ == '__main__':
    try:
        rospy.init_node('BaseController')
        MotorController()
        time.sleep(.01)
        rospy.spin()
    except rospy.ROSInterruptException: pass

