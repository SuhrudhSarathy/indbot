#!/usr/bin/env python

try :
    import rospy 
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import Point, PoseStamped, Quaternion, Pose
    from std_msgs.msg import Header
    
except:
    raise ImportError
import numpy as np 
from rrt import RRT, Node



class Manager():

    def __init__(self):
        self.position = Point()
        self.path = Path()
        self.obstacles = []
        self.start = Point()
        self.goal = Point()

        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        #self.obstacles_sub = rospy.Subscriber('/obstacles', PoseArray, self.__obstacle_sub)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_update)
        self.not_reached = True
        self.path_planner = RRT(1, 1000)
        self.path_points = []
    
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
            self.path_points = self.path_planner.get_path(Node(self.start.x, self.start.y), Node(self.goal.x, self.goal.y), self.obstacles)
            print('planner called')
            print(self.path_points)
            self.publish_path()
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

if __name__ == '__main__':
    rospy.init_node('dynManager')
    rate = rospy.Rate(10)
    
    manager = Manager()
    
    manager.start = Point(0, 0, 0)
    manager.goal = Point(1, 2, 0)

    rospy.spin()


    