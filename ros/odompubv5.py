#!/usr/bin/env python
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from odompackage.msg import EncoderTick

Left_Ticks=0
Right_Ticks=0

def tickcallback(msg):
    global Left_Ticks
    global Right_Ticks
    Left_Ticks = msg.left
    Right_Ticks = msg.right
    rospy.loginfo("I heard")

ex_Left_Ticks = 0
ex_Right_Ticks = 0


rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
tick_sub =rospy.Subscriber("TicksPublisher",EncoderTick,tickcallback)
odom_broadcaster = tf.TransformBroadcaster()




current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10.0)



R = 0.0325        #//Wheel Radius
N = 40            #// Number of Ticks Per revolution
L = 0.125         #//Distance between left and right wheels

x = 0.0
y = 0.0
th = 0.0

vx =  0.0
vy =  0.0
vth =  0.0

dc = 0.0
dr = 0.0
dl = 0.0
dt = 0.0
dx = 0.0
dy = 0.0
dtheta = 0.0




while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    L_delta_Tick = Left_Ticks - ex_Left_Ticks
    R_delta_Tick = Right_Ticks - ex_Right_Ticks
    dt = (current_time - last_time).to_sec()
    #rospy.loginfo(dt)
        #rospy.loginfo(Right_Ticks)
        #rospy.loginfo(Left_Ticks)

    dl = 2 * pi * R * L_delta_Tick / N
    dr = 2 * pi * R * R_delta_Tick / N
    dc = (dl + dr) / 2
    dtheta = (dr - dl) / L
        #dx = cos(th) * dc
        #dy = sin(th) * dc
        # compute odometry in a typical way given the velocities of the robot
        #delta_x = (vx * cos(th) - vy * sin(th)) * dt
        #delta_y = (vx * sin(th) + vy * cos(th)) * dt
        #delta_th = vth * dt
    dx=cos(dtheta)*dc
    dy=-sin(dtheta)*dc
    x=x + (cos(th)*dx - sin(th)*dy)
    y=y + (sin(th)*dx + cos(th)*dy)
    th=th+dtheta
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = sin(th / 2)
    quaternion.w = cos(th / 2)
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        current_time,
        "base_link",
        "odom"
    )

        # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0
    odom.pose.pose.orientation = quaternion
    vx=dx/dt
    vy=dy/dt
    v=dc/dt
    vth=dtheta/dt
    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x=v
    odom.twist.twist.linear.y=0
    odom.twist.twist.angular.z=vth

    odom_pub.publish(odom)

    ex_Left_Ticks=Left_Ticks
    ex_Right_Ticks = Right_Ticks
    last_time = current_time
    r.sleep()


