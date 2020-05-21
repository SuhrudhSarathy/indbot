#!/usr/bin/env python

try :
    import rospy 
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import Point, PoseStamped, Quaternion, Pose, Point32
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Header
    
except:
    raise ImportError
import numpy as np 
from rrt import RRT, Node
from shapely.geometry import LineString

OBST_THRES = 0.25

class Manager():
    '''
        Main manager class that takes care of calling the path planner
        Also takes care of updating the nextWay point for the bot to move to in the path
    '''

    def __init__(self):

        #Initialising messages and variables
        self.position = Point()
        self.path = Path()
        self.obstacles = []
        self.start = Point()
        self.goal = Point()
        self.scan = LaserScan()

        #Publishers and Subscribers
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_update)      
        self.scan_pub = rospy.Subscriber('/scan', LaserScan, self.laser_sub)    

        #Flag to check if final goal is reached
        self.not_reached = True

        #Initialising path planner
        self.path_planner = RRT(1, 10000)
        self.path_points = []
        
        
    def __odom_update(self, msg):
        '''
            Callback function for odometry
        '''
        self.position = msg.pose.pose.position

        # call get path function
        self.get_path()

    def laser_sub(self, msg):
        '''
            Callback function for laserScan
        '''
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.ranges = msg.ranges

        self.convert_to_points()
        self.publish_polygons()
        
###---------------------------------------Navigation Functions----------------------------------------###       
    def get_path(self):
        '''
            Funtions that checks necessary conditions and calls path
            Note:
                Future Updates: Dynamic Collision Checking
        '''
        try:
            if abs(self.position.x - self.goal.x) < 0.1 and abs(self.position.y - self.goal.y) < 0.1:
                self.not_reached = False
        except:
            pass
        if len(self.path_points) == 0 and self.not_reached:
            try:
                rospy.loginfo('Path Planner Called')
                self.path_points = self.path_planner.get_path(Node(self.position.x, self.position.y), Node(self.goal.x, self.goal.y), self.linestrings)
                self.publish_path()
            except Exception as err:
                rospy.logerr(err)
            
        elif self.not_reached:
            self.publish_path()
        else :
            self.publish_path()
            rospy.loginfo("-"*10 + 'End-Reached' + '-'*10)
        

    def publish_path(self):
        '''
            Publishes the path given out byt the path planner
        '''
        poses = []
        self.path = Path()
        self.path.header = Header(frame_id = 'odom')
        for point in self.path_points:
            pose = PoseStamped(header = Header(frame_id = 'odom', stamp = rospy.Time.now()), pose = Pose(position = Point(point[0], point[1], 0), orientation = Quaternion(0, 0, 0, 1)))
            poses.append(pose)
        self.path.poses = poses

        self.path_pub.publish(self.path)
    ### ----------------------------- Functions for LaserScan Processing-----------------------------###

    def convert_to_points(self):
        '''
            Converts the LaserScan to Points wrt to Bot Frame
        '''
        self.obstacles = []
        angle_min = self.angle_min
        angle_increment = self.angle_increment
        current_index = 0
        self.obstacles = [[self.convert_to_rect(self.ranges[0], angle_min)]]
        for i in range(1, len(self.ranges)):
            #Loop through the ranges
            if self.ranges[i] < 1000:
                # If the diff(y_coords) of two points is less than the given threshold the new point belongs to the same obstcale
                if abs(self.convert_to_rect(self.ranges[i], angle_min).y - self.convert_to_rect(self.ranges[i-1], angle_min - angle_increment).y) < OBST_THRES:
                    self.obstacles[current_index].append(self.convert_to_rect(self.ranges[i], angle_min))
                else:
                    # Else update the current index and associate new point to a new polygon
                    self.obstacles.append([])
                    current_index += 1
                    self.obstacles[current_index].append(self.convert_to_rect(self.ranges[i], angle_min))
            else:
                pass
            
            # Finally add the angle increment 
            angle_min += angle_increment
            
    def publish_polygons(self):
        '''
            Publishes Polygons as LineStrings using a very naive algorithm
        '''
        linestrings = []
        for obstacle in self.obstacles:
            try:
                linestring = LineString([(p.x, p.y) for p in obstacle])
                linestrings.append(linestring)
            except:
                pass
            self.linestrings = linestrings    
    

    def convert_to_rect(self, r, theta):
        '''
            Used to convert from (r, theta) to (x, y)
        '''
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return Point32(x, y, 0)

    


if __name__ == '__main__':
    # Initialise node
    rospy.init_node('dynManager')
    rate = rospy.Rate(10)
    
    # Manager Instance
    manager = Manager()
    
    manager.start = Point(0, 0, 0)
    manager.goal = Point(2, 2, 0)
    rospy.loginfo('Manager Initiated')
    rospy.spin()


    