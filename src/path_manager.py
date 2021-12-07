#!/usr/bin/env python

# path_manager:
# - subscribes to currentTarget msg
# - broadcasts currentWaypoint msg
# - calls final_a_star pathfinding

import rospy
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PointStamped
from std_msgs.msg import Header
import tf2_ros

class PathManager:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


if __name__ == '__main__':
    rospy.init_node('path_manager')
    #pl = PoseListener()
    rospy.spin()

