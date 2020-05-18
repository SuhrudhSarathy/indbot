#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Point, Quaternion
from tf.transformations import euler_from_quaternion as efq 

import numpy as np 
from collections import namedtuple

Orientation = namedtuple('Orientation', ['roll', 'pitch', 'yaw'])
Gains = namedtuple('Gains', ['kp', 'kd', 'ki'])

DISTMIN = 0.1
MAXX = 0.22
MAXANG = 2
DIST_THRES = 0.2


class Controller():
    '''
        Main controller class
    '''

    def __init__(self):
        
        # Define the messages
        self.velocity = Twist()
        self.position = Point()
        self.orientation = Orientation(0, 0, 0)
        self.path = Path()
        self.path_points = []
        self.current_index = 0
        self.nextWay = None
        self.goal = None
        self.start = None
        self.goal = None
        self.goal_reached = False
        
        # Initialize subs and pubs
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_sub = rospy.Subscriber('/path', Path, self.__path_sub)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_sub)

        # Parameters for PID tuning
        self.x_gain = Gains(1, 0, 0)
        self.ang_gain = Gains(0.3, 0, 0)

    def __odom_sub(self, msg):
        self.position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        orientatioN = efq([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.orientation = Orientation(orientatioN[0], orientatioN[1], orientatioN[2])
        self._set_goal()
          
        
    def __path_sub(self, msg):
        self.path = msg
        poses = self.path.poses
        self.path_points = [Point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in poses]
     
    def _set_goal(self):

        if abs(self.final_goal.x - self.position.x) < DISTMIN and (self.final_goal.y - self.position.y) < DISTMIN:
            self.goal_reached = True
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
            self.velocity.linear.x, self.velocity.angular.z = 0, 0
            self.vel_pub.publish(self.velocity)
            print('-------Completed Path-------')

        
        
    def vel_constraint(self, velocity, dir):
        '''
            sets the velocity constraints
        '''
        if dir.lower() == 'x':
            if velocity > MAXX:
                velocity = MAXX
            elif velocity < - MAXX:
                velocity = MAXX

            else:
                velocity = velocity

        elif dir.lower() == 'ang':
            if velocity > MAXANG:
                velocity = MAXANG
            elif velocity < - MAXANG:
                velocity = MAXANG

            else:
                velocity = velocity

        return velocity

    def _move_bot(self):
        '''
            handles all the velocity commands to be published
        '''
        ang_goal = np.arctan2(self.goal.y - self.position.y, self.goal.x - self.position.x)
        x_error = np.sqrt((self.goal.x - self.position.x) ** 2 + (self.goal.y - self.position.y) ** 2)
        ang_error = ang_goal - self.orientation.yaw

        xdiff, angdiff, xintegral, angintegral = 0, 0, 0, 0

        if self.move_to_goal:
            #while abs(x_error) > DISTMIN :

            x_error = np.sqrt((self.goal.x - self.position.x) ** 2 + (self.goal.y - self.position.y) ** 2)
            ang_error = ang_goal - self.orientation.yaw
    
            if abs(ang_error) < DIST_THRES:
                ang_vel = self.ang_gain.kp * ang_error + self.ang_gain.kd * angdiff + self.ang_gain.ki * angintegral
                lin_vel = self.x_gain.kp * x_error + self.x_gain.kd * xdiff + self.x_gain.ki * xintegral
                ang_vel = max(-MAXANG, min(ang_vel, MAXANG))
                lin_vel = max(0, min(lin_vel, MAXX))

                xdiff = x_error - xdiff
                xintegral += x_error

                angdiff = ang_error - angdiff
                angintegral += ang_error
                
            else:
                ang_vel = self.ang_gain.kp * ang_error + self.ang_gain.kd * angdiff + self.ang_gain.ki * angintegral
                ang_vel = max(-MAXANG, min(ang_vel, MAXANG))
                angdiff = ang_error - angdiff
                angintegral += ang_error
                lin_vel = 0
                
            
            self.velocity.linear.x = lin_vel
            self.velocity.angular.z = ang_vel
            self.vel_pub.publish(self.velocity)
        else:
        
            self.velocity.linear.x = 0
            self.velocity.angular.z = 0
            self.vel_pub.publish(self.velocity)
            print('Waypoint Reached')


if __name__ == '__main__':
    rospy.init_node('omnicontroller')
    rate = rospy.Rate(10)

    controller = Controller()
    controller.final_goal = Point(1, 2, 0)
    controller.start = Point(0, 0, 0)
    

    rospy.spin()
        

    


