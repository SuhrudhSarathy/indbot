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



class Manager():

    def __init__(self):
        self.position = Point()
        self.path = Path()
        self.obstacles = []
        self.start = Point()
        self.goal = Point()
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_update)
        self.not_reached = True
        self.path_planner = RRT(1, 10000)
        self.path_points = []
        #self.linestrings = []
        self.scan = LaserScan()

        self.scan_pub = rospy.Subscriber('/scan', LaserScan, self.laser_sub)
        
    
    def __odom_update(self, msg):
        self.position = msg.pose.pose.position
        self.get_path()
        
        
    def get_path(self):
        try:
            if abs(self.position.x - self.goal.x) < 0.1 and abs(self.position.y - self.goal.y) < 0.1:
                self.not_reached = False
        except:
            pass
        if len(self.path_points) == 0 and self.not_reached:
            try:
                #print(self.linestrings)
                self.path_points = self.path_planner.get_path(Node(self.start.x, self.start.y), Node(self.goal.x, self.goal.y), self.linestrings)
                self.publish_path()
            except Exception as err:
                print(err)
            
        elif self.not_reached:
            self.publish_path()
        else:
            print("End_reached")

    def publish_path(self):
        poses = []
        self.path = Path()
        self.path.header = Header(frame_id = 'odom')
        for point in self.path_points:
            pose = PoseStamped(header = Header(frame_id = 'odom', stamp = rospy.Time.now()), pose = Pose(position = Point(point[0], point[1], 0), orientation = Quaternion(0, 0, 0, 1)))
            poses.append(pose)
        self.path.poses = poses

        self.path_pub.publish(self.path)
    
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
        linestrings = []
        for obstacle in self.obstacles:
            try:
                linestring = LineString([(p.x, p.y) for p in obstacle])
                linestrings.append(linestring)
            except:
                pass
            self.linestrings = linestrings
        
    
        


    def convert_to_rect(self, r, theta):
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return Point32(x, y, 0)

    


if __name__ == '__main__':
    rospy.init_node('dynManager')
    rate = rospy.Rate(10)
    
    
    manager = Manager()
    
    manager.start = Point(0, 0, 0)
    manager.goal = Point(2, 2, 0)

    rospy.spin()


    