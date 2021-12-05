#!/usr/bin/env python
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from points_to_rviz import draw_points
from visualization_msgs.msg import Marker


class map_odom_listener(object):
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.map_odom = rospy.Subscriber('/map_odom', Odometry, self.rviz_rob_marker)

    def rviz_rob_marker(self):
	pub = rospy.Publisher('/marker', Marker, queue_size=2)
	print(self.map_odom)
	
	
if __name__ == '__main__':
    rospy.init_node('test_rviz_write_node')
    pl = map_odom_listener()
    rospy.spin()
