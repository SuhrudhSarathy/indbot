#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, Point, Quaternion
from tf.transformations import euler_from_quaternion as efq 

import numpy as np 
from collections import namedtuple

Orientation = namedtuple('Orientation', ['roll', 'pitch', 'yaw'])
Gains = namedtuple('Gains', ['kp', 'kd', 'ki'])

DISTMIN = 0.1
MAXX = 0.25
MAXY = 0.25

class Controller():
    '''
        Main controller class
    '''

    def __init__(self):
        
        # Define the messages
        self.velocity = TwistStamped()
        self.position = Point()
        self.orientation = Orientation(0, 0, 0)
        self.path = Path()
        self.path_points = []
        self.index = 0
        self.nextWay = None
        self.goal_reached = False
        self.goal = None
        self.current_index = 0
        self.move_to_goal = False

        # Initialize subs and pubs
        self.vel_pub = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=10)
        self.path_sub = rospy.Subscriber('/path', Path, self.__path_sub)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_sub)

        # Parameters for PID tuning
        self.x_gain = Gains(1, 0, 0)
        self.y_gain = Gains(1, 0, 0)

    def __odom_sub(self, msg):
        self.position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        orientatioN = efq([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.orientation = Orientation(orientatioN[0], orientatioN[1], orientatioN[2])
        
        
        
        
    def __path_sub(self, msg):
        self.path = msg
        poses = self.path.poses
        self.path_points = [Point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in poses]
        self.set_goal()
        

    def set_goal(self):
        '''
            Sets next waypoint fot the bot to travel to
        '''
        try:
            self.final_goal = self.path_points[-1]
            if np.sqrt((self.final_goal.x - self.position.x)**2 + (self.position.y - self.final_goal.y)**2) < 0.1:
                rospy.loginfo('final_goal_reached')
                self.goal_reached = True
                self.move_to_goal = False
                self.current_index = 0  

            if len(self.path_points) != 0 and not self.goal_reached:
                self.nextWay = self.path_points[self.current_index]
                print((self.nextWay.x, self.nextWay.y), self.current_index)
                if abs(self.nextWay.x - self.position.x) < DISTMIN and abs(self.nextWay.y - self.position.y) < DISTMIN:
                    self.move_to_goal = False                    
                    self.current_index += 1
                    print('popped')
                else :
                    self.goal = self.nextWay
                    self.move_to_goal = True
                    self._move_bot()
                    print('next goal set')
            elif self.goal_reached:
                self.velocity.twist.linear.x, self.velocity.twist.angular.z = 0, 0
                self.vel_pub.publish(self.velocity)
                print('-------Completed Path-------')

        except Exception as err:
            rospy.logwarn(err)
        
        
    def vel_constraint(self, velocity, dir):
        '''
            sets tyhe velocity constraints
        '''
        if dir.lower() == 'x':
            if velocity > MAXX:
                velocity = MAXX
            elif velocity < - MAXX:
                velocity = MAXX

            else:
                velocity = velocity

        elif dir.lower() == 'y':
            if velocity > MAXY:
                velocity = MAXY
            elif velocity < - MAXY:
                velocity = MAXY

            else:
                velocity = velocity

        return velocity

    def _move_bot(self):
        '''
            handles all the velocity commands to be published
        '''
        x_error = abs(self.position.x - self.nextWay.x)
        y_error = abs(self.position.y - self.nextWay.y)

        xdiff, ydiff, xintegral, yintegral = 0, 0, 0, 0

        if self.move_to_goal:

            x_error = abs(self.position.x - self.nextWay.x)
            y_error = abs(self.position.y - self.nextWay.y) 

            velx = self.x_gain.kp * x_error + self.x_gain.kd * xdiff + self.x_gain.ki * xintegral
            vely = self.y_gain.kp * y_error + self.y_gain.kd * ydiff + self.y_gain.ki * yintegral

            velx, vely = self.vel_constraint(velx, 'x'), self.vel_constraint(vely, 'y')

            xdiff = x_error - xdiff
            ydiff = y_error - ydiff

            xintegral += x_error
            yintegral += y_error
            self.velocity.twist.linear.x = velx
            self.velocity.twist.linear.y = vely

            self.vel_pub.publish(self.velocity)
            print('vel published')

        else:
        
            self.velocity.twist.linear.x = 0
            self.velocity.twist.angular.z = 0
            self.vel_pub.publish(self.velocity)
            print('Waypoint Reached')


if __name__ == '__main__':
    rospy.init_node('omnicontroller')
    rate = rospy.Rate(10)

    controller = Controller()
    

    rospy.spin()
        

    


