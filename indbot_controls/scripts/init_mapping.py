#!/usr/bin/env python

import rospy
import json
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

DIST_THRES = 3

class Mapper():

    '''
        This class is used to map and store viable points as nodes
        for further path planning algos
    '''
    def __init__(self):

        #init messages
        self.position = Point()
        self.nodes = []

        self.odom_sub = rospy.Subscriber('odom', Odometry, self._odom_callback)

    
    def _odom_callback(self, msg):
        self.position = msg.pose.pose.position
        if len(self.nodes) == 0:
            self.nodes = [(self.position.x, self.position.y)]
        else:
            last_node = self.nodes[-1]
            if np.linalg.norm(np.array(last_node) - np.array((self.position.x, self.position.y))) >= DIST_THRES:
                self.nodes.append((self.position.x, self.position.y))
                with open('/home/suhrudh/indbot_ws/src/indbot_controls/scripts/nodes.json', 'w') as file:
                    json.dump(self.nodes, file)
        
    
        

if __name__ == '__main__':
    rospy.init_node('mapper')
    rate = rospy.Rate(10)

    mapper = Mapper()

    rospy.spin()







