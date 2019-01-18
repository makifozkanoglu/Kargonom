#! /usr/bin/env python
from __future__ import division

import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import sin, cos
from diff_drive.pose import Pose
from diff_drive import odometry
from odompackage.msg import EncoderTick

class OdometryNode:

    def __init__(self):
        self.odometry = odometry.Odometry()


        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tfPub = TransformBroadcaster()

        rospy.init_node('diff_drive_odometry')
        self.nodeName = 'diff_drive_odometry'  # rospy.get_name()
        rospy.loginfo("{0} started".format(self.nodeName))
        rospy.Subscriber("/TicksPublisher", EncoderTick, self.TickCallback)
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.on_initial_pose)

        self.ticksPerMeter = 173.913  # int(rospy.get_param('~ticks_per_meter'))
        self.wheelSeparation = float(0.125)  # float(rospy.get_param('~wheel_separation'))
        self.rate = float(10.0)  # float(rospy.get_param('~rate', 10.0))
        self.baseFrameID = 'base_link'  # rospy.get_param('~base_frame_id', base_link)
        self.odomFrameID = 'odom'  # rospy.get_param('~odom_frame_id', 'odom')
        self.encoderMin = -32768  # int(rospy.get_param('~encoder_min', -32768))
        self.encoderMax = 32767  # int(rospy.get_param('~encoder_max', 32767))
        self.right = 0
        self.left = 0
        self.odometry.setWheelSeparation(self.wheelSeparation)
        self.odometry.setTicksPerMeter(self.ticksPerMeter)
        self.odometry.setEncoderRange(self.encoderMin, self.encoderMax)
        self.now = rospy.get_rostime()
        self.odometry.setTime(self.now.to_sec())

    def main(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        self.odometry.updateRightWheel(self.right)
        self.odometry.updateLeftWheel(self.left)
        self.now = rospy.get_rostime()

        sec=self.now.to_sec()
        self.odometry.updatePose(sec)
        pose = self.odometry.getPose()
        rospy.loginfo('Published ')

        q = quaternion_from_euler(0, 0, pose.theta)        # q = quaternion_from_euler(0, 0, pose.theta)
        self.tfPub.sendTransform(
            (pose.x, pose.y, 0),
            (q[0], q[1], q[2], q[3]),
            self.now,
            self.baseFrameID,
            self.odomFrameID
        )
        odom = Odometry()
        odom.header.stamp = self.now
        odom.header.frame_id = self.odomFrameID
        odom.child_frame_id = self.baseFrameID
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = pose.xVel
        odom.twist.twist.angular.z = pose.thetaVel
        self.odomPub.publish(odom)

    def on_initial_pose(self,msg): #(self, msg):
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        pose = Pose()
        pose.x = msg.pose.pose.position.x
        pose.y = msg.pose.pose.position.y
        pose.theta = yaw
        rospy.loginfo('Setting initial pose to ')
        self.odometry.setPose(pose)
    def TickCallback(self, msg):
        self.right=msg.right
        self.left= msg.left


if __name__ == '__main__':
    try:
        node = OdometryNode()
        node.main()
    except rospy.ROSInterruptException:
        pass