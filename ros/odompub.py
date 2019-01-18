#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from odompackage.msg import EncoderTick
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3




R = 0.0325        #//Wheel Radius
N = 40            #// Number of Ticks Per revolution
L = 0.125         #//Distance between left and right wheels


rospy.init_node('odometry_publisher')
Left_Ticks=0
Right_Ticks=0

ex_Left_Ticks = 0
ex_Right_Ticks = 0

def tickcallbak(msg):
    Left_Ticks = msg.left
    Right_Ticks = msg.right
    rospy.loginfo("I heard")

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
tick_sub =rospy.Subscriber("TicksPublisher",EncoderTick,tickcallbak)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
theta = 0.0

vx = 0.0
vy = 0.0
vth = 0.0


dc = 0.0
dr = 0.0
dl = 0.0
dt = 0.0
dx = 0.0
dy = 0.0
dtheta = 0.0





current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(1.0)
while not rospy.is_shutdown():
    L_delta_Tick = Left_Ticks - ex_Left_Ticks
    R_delta_Tick = Right_Ticks - ex_Right_Ticks

    dl = 2 * pi * R * L_delta_Tick / N
    dr = 2 * pi * R * R_delta_Tick / N
    dc = (dl + dr) / 2
    dtheta = (dr - dl) / L
    dx = cos(theta) * dc
    dy = sin(theta) * dc

    x += dx
    y += dy
    theta += dtheta


    current_time = rospy.Time.now()
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    vx=dx/dt
    vy=dy/dt
    vth=dtheta/dt
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    ex_Left_Ticks=Left_Ticks
    ex_Right_Ticks = Right_Ticks
    last_time = current_time
    rospy.spin()
r.sleep()