#!/usr/bin/env python
import math
from math import sin, cos, pi

import rospy
import tf
from odompackage.msg import EncoderTick
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class OdometryPublisher(object):
    def __init__(self):

        self.ex_Left_Ticks = 0
        self.ex_Right_Ticks = 0
        self.Left_Ticks=0
        self.Right_Ticks=0

        self.R = 0.0325  # //Wheel Radius
        self.N = 20  # // Number of Ticks Per revolution
        self.L = 0.125  # //Distance between left and right wheels

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.dt=0
        self.dtheta=0

        self.vx=0
        self.vy=0
        self.v=0
        self.vth=0

        self.dx=0
        self.dy=0

        rospy.init_node('odometry_publisher')
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
        self.tick_sub = rospy.Subscriber("/TicksPublisher", EncoderTick, self.tickcallback)
        self.odom_broadcaster = tf.TransformBroadcaster()
    def PoseCalc(self,R_delta_Tick,L_delta_Tick):

        dl = 2 * pi * self.R * L_delta_Tick / self.N
        dr = 2 * pi * self.R * R_delta_Tick / self.N
        dc = (dl + dr) / 2
        self.v = dc / self.dt
        self.dtheta = (dr - dl) / self.L
        self.dx = cos(self.dtheta) * dc
        self.dy = -sin(self.dtheta) * dc
        self.x += self.dx * cos(self.theta) - self.dy * sin(self.theta)
        self.y += self.dx * sin(self.theta) + self.dy * cos(self.theta)
        self.theta += self.dtheta
    def VelocityCalc(self):
        self.vx = self.dx / self.dt
        self.vy = self.dy / self.dt
        self.vth = self.dtheta / self.dt

    def tickcallback(self,msg):
        self.current_time = rospy.Time.now()
        self.dt = (self.current_time - self.last_time).to_sec()
        if self.dt>20:
            Left_Ticks = msg.left
            Right_Ticks = msg.right
            rospy.loginfo(msg.left)
            rospy.loginfo(msg.right)
            rospy.loginfo("I heard")
            rospy.loginfo(self.Left_Ticks)
            rospy.loginfo(self.Left_Ticks)
            R_delta_Tick = self.Right_Ticks - self.ex_Right_Ticks
            L_delta_Tick = self.Left_Ticks - self.ex_Left_Ticks
            self.PoseCalc(R_delta_Tick,L_delta_Tick)
            self.VelocityCalc()
            # since all odometry is 6DOF we'll need a quaternion created from yaw
            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0),
                (0, 0, self.theta, 0), self.current_time, "base_link", "odom")

            # next, we'll publish the odometry message over ROS
            self.odom.header.stamp = self.current_time
            # set the position
            self.odom.pose.pose.position.x = self.x
            self.odom.pose.pose.position.y = self.y
            self.odom.pose.pose.position.z = 0
            self.odom.pose.pose.orientation.z = self.theta
            # set the velocity
            self.odom.twist.twist.linear.x = self.v
            self.odom.twist.twist.linear.y = 0
            self.odom.twist.twist.angular.z = self.vth

            # publish the message
            self.odom_pub.publish(self.odom)

            ex_Left_Ticks = Left_Ticks
            ex_Right_Ticks = Right_Ticks
            self.last_time = self.current_time

rospy.init_node('odometry_publisher')
if __name__ == '__main__':
    try:
        pub=OdometryPublisher()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
