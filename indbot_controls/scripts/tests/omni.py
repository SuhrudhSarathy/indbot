#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped

rospy.init_node('controller_test')
vel_pub = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=10)
vel = TwistStamped()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    vel.twist.linear.x = 1 
    vel.twist.linear.y = 1 

    vel_pub.publish(vel)

    rate.sleep()