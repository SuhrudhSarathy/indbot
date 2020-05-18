#!/usr/bin/env python
import rospy
import numpy as np 
import matplotlib.pyplot as plt 

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry

from shapely.geometry import LineString

class Detector():
    '''
        Main Obstacle Detector class
    '''
    def __init__(self):
        self.obstacles = []
        self.linestrings = []
        self.scan = LaserScan()

        self.scan_pub = rospy.Subscriber('/scan', LaserScan, self.laser_sub)
        
       

    def laser_sub(self, msg):
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.ranges = msg.ranges

        self.convert_to_points()
        self.publish_polygons()
        


    def convert_to_points(self):
        self.obstacles = []
        angle_min = self.angle_min
        angle_increment = self.angle_increment
        current_index = 0
        self.obstacles = [[self.convert_to_rect(self.ranges[0], angle_min)]]
        for i in range(1, len(self.ranges)):
            if self.ranges[i] < 1000:
                if abs(self.convert_to_rect(self.ranges[i], angle_min).y - self.convert_to_rect(self.ranges[i-1], angle_min - angle_increment).y) < 0.25:
                    self.obstacles[current_index].append(self.convert_to_rect(self.ranges[i], angle_min))
                else:
                    self.obstacles.append([])
                    current_index += 1
                    self.obstacles[current_index].append(self.convert_to_rect(self.ranges[i], angle_min))
            else:
                pass
            
            angle_min += angle_increment
            
    def publish_polygons(self):
        self.linestrings = []
        for obstacle in self.obstacles:
            try:
                linestring = LineString([(p.x, p.y) for p in obstacle])
                self.linestrings.append(linestring)
            except:
                pass
    
    def get_obst(self):
        


    def convert_to_rect(self, r, theta):
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return Point32(x, y, 0)


