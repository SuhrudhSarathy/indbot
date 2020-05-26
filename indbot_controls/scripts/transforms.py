#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros, tf2_geometry_msgs

from geometry_msgs.msg import Twist, PointStamped, Point
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node('check_transforms')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10)
    point = PointStamped(header=Header(frame_id = 'odom'), point = Point(2, 2, 0))
    while not rospy.is_shutdown():

        try:
            trans = tfBuffer.lookup_transform('base_footprint', 'odom', rospy.Time())
            
            point_transformed = tf2_geometry_msgs.do_transform_point(point, trans)
            print(point_transformed.point.x, point_transformed.point.y, point_transformed.header.frame_id)

        except:
            continue

        rate.sleep()

        

