#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion as efq 
from collections import namedtuple

Orientation = namedtuple('Orientation', ['roll', 'pitch', 'yaw'])



class Controller():

    def __init__(self):

        #init messages
        self.position = Point()
        self.orientation = Orientation(0, 0, 0)
        self.velocity = Twist()
        self.goal = Point()

        #init pubs and subs
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
    
        #Vars for controller
        self.DISTMIN = 0.1
        self.ANG_THRES = 0.25

        self.max_lin_vel = 0.22
        self.max_ang_vel = 1.0
        self.min_lin_vel = 0.1
        self.min_ang_vel = 0.0

        self.kp_lin = 1.0
        self.kp_ang = 0.3

        self.move_to_goal = False
    def _odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation0 = efq([msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w])
        self.orientation = Orientation(self.orientation0[0], self.orientation0[1], self.orientation0[2])

    def goto_func(self):
        lin_error = 0.
        ang_error = 0.

        lin_vel = 0.
        ang_vel = 0. 

        lin_error = math.sqrt((self.goal.x - self.position.x)**2 + (self.position.y - self.goal.y)**2)
        angle_error = math.atan2(self.goal.y - self.position.y, self.goal.x - self.position.x) - self.orientation.yaw

        print 'lin_error: ', lin_error, ' ang_error: ', ang_error

        while lin_error > self.DISTMIN:

            lin_error = math.sqrt((self.goal.x - self.position.x)**2 + (self.position.y - self.goal.y)**2)
            angle_error = math.atan2(self.goal.y - self.position.y, self.goal.x - self.position.x) - self.orientation.yaw

            print 'lin_error: ', lin_error, ' ang_error: ', ang_error

            if abs(angle_error) > self.ANG_THRES:
                lin_vel = 0.
            else:
                lin_vel = max(self.min_lin_vel, min(self.kp_lin * lin_error, self.max_lin_vel))

            ang_vel = min(self.kp_ang * angle_error, self.max_ang_vel)

            print 'velx', lin_vel, 'ang_vel', ang_vel

            self.velocity.linear.x = lin_vel
            self.velocity.angular.z = ang_vel

            self.vel_pub.publish(self.velocity)
        
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0

        self.vel_pub.publish(self.velocity)
        print "Goal reached"


    


if __name__ == '__main__':
    rospy.init_node('controller')
    rate = rospy.Rate(10)
    controller = Controller()
    x = input()
    y = input()
    controller.goal = Point(x, y, 0)
    controller.goto_func()
    rospy.spin()    
    
                
