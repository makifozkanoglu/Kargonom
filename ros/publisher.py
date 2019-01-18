#! /usr/bin/env python
#license removed for brevity
import rospy
from std_msgs.msg import String
from odompackage.msg import EncoderTick

def talker():
    pub = rospy.Publisher('TicksPublisher', EncoderTick, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    msg = EncoderTick()
    msg.left=0
    msg.right=0
    rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        msg.left+=5+1+1+1+1+1+1
        msg.right+=5
        pub.publish(msg)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass