#!/usr/bin/env python

# path_manager:
# - subscribes to CurrentTarget msg
# - broadcasts CurrentWaypoint msg
# - calls final_a_star pathfinding

import rospy
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PointStamped
from std_msgs.msg import Header, String
import tf2_ros
from final_a_star import floodFill
from nav_msgs.msg import Odometry


class PathManager:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_sub = rospy.Subscriber('/Current_Target', String, self.current_target_interpret)
        self.pos_sub = rospy.Subscriber('map_odom', Pose, self.current_pos_interpret)
        self.waypoint_pub = rospy.Publisher('/Current_Waypoint', String, queue_size=10)
        self.TargetTuple = (1, 0)
        self.cur_loc = (0, 0)
        self.new_target_flag = 1
        self.waypoint_list = 0
        self.waypoint_index = 0
        self.cur_waypoint = 0
	self.fill = floodFill()	


    def waypoint_manager(self):
        """"""
        # check to see if a different target has been published
        if self.new_target_flag == 1:
            self.new_target_flag = 0
            self.waypoint_index = 0
            self.waypoint_list = self.call_a_star(self.cur_loc, self.TargetTuple)
	    self.waypoint_list

        # code to check if sufficiently close to a waypoint to shift to new waypoint
        #elif self.waypoint_list(self.waypoint_index) - self.cur_loc <

        self.cur_waypoint = self.waypoint_list[self.waypoint_index]
	self.publish_waypoint()

    def current_target_interpret(self, target_msg):
        """reads the current target public message and if it is different
        assigns it as the current target"""
        if self.TargetTuple != tuple([target_msg.position.x, target_msg.position.y]):
            self.new_target_flag = 1
            self.TargetTuple = tuple([target_msg.position.x, target_msg.position.y])

    def current_pos_interpret(self, pos_msg):
        self.cur_loc = tuple([pos_msg.position.x, pos_msg.position.y])

    def publish_waypoint(self):
        print("waypoint published!")
        self.waypoint_pub.publish(String(str(self.cur_waypoint)))

    def call_a_star(self, start, end):
        return self.fill.flood_fill_do(start, end)



if __name__ == '__main__':
    rospy.init_node('path_manager')
    PM = PathManager()
    PM.waypoint_manager()
    rospy.spin()

