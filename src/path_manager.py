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
import numpy as np


class PathManager:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
	self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_sub = rospy.Subscriber('/Current_Target', Pose, self.current_target_interpret)
        self.pos_sub = rospy.Subscriber('/map_odom', Pose, self.current_pos_interpret)
	self.waypoint_pub = rospy.Publisher('/Current_Waypoint', Pose, queue_size=10)
        self.TargetTuple = (1, 0)
        self.cur_loc = (0, 0)
        self.new_target_flag = 1
        self.waypoint_list = 0
        self.waypoint_index = 0
        self.cur_waypoint = 0
	self.fill = floodFill()
	self.waypoint_pose = Pose()	


    def waypoint_manager(self):
    	#rospy.init_node('path_manager')
	r = rospy.Rate(3) # 10hz 	

	while not rospy.is_shutdown():

	    #rospy.init_node('path_manager')
	    #self.tf_buffer = tf2_ros.Buffer()
	    #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            #self.target_sub = rospy.Subscriber('/Current_Target', Pose, self.current_target_interpret)
            #self.pos_sub = rospy.Subscriber('/map_odom', Pose, self.current_pos_interpret)
            #self.waypoint_pub = rospy.Publisher('/Current_Waypoint', Pose, queue_size=10)

	    #self.waypoint_pose = Pose()

            # check to see if a different target has been published
            if self.new_target_flag == 1:
                self.new_target_flag = 0
                self.waypoint_index = 0
		astar_start = tuple([self.cur_loc[1],self.cur_loc[1]])
		astar_end = tuple([self.TargetTuple[1], self.TargetTuple[0]])
		print("running A*")
                self.waypoint_list = self.call_a_star(astar_start, astar_end)
	
	    # Calculate distance to waypoint. If small, shift to next waypoint
	    # NEED TO CHECK X Y ORDER IS CORRECT
	    x_dist = self.waypoint_list[self.waypoint_index][0]-self.cur_loc[0]
	    y_dist = self.waypoint_list[self.waypoint_index][1]-self.cur_loc[1]
	    print('length ',len(self.waypoint_list))
	    if np.sqrt(x_dist**2+y_dist**2)<.2:
		if (self.waypoint_index+1) < len(self.waypoint_list):
		    self.waypoint_index += 1
		    print(self.waypoint_index)
		else:
		    print('Reached last waypoint!')
		    # do additional actions here...

            # code to check if sufficiently close to a waypoint to shift to new waypoint
            #elif self.waypoint_list(self.waypoint_index) - self.cur_loc <

            self.cur_waypoint = self.waypoint_list[self.waypoint_index]
	    self.waypoint_pose.position.x = self.cur_waypoint[0]
	    self.waypoint_pose.position.y = self.cur_waypoint[1]
	    #print(self.cur_waypoint)
	    self.publish_waypoint()

	    #print("waypoing_man looped")
	    r.sleep()
	rospy.spin()


    def current_target_interpret(self, target_msg):
        """reads the current target public message and if it is different
        assigns it as the current target"""
	#print("ran curr tar interp")
	#print(target_msg.position.x)
        if self.TargetTuple != tuple([target_msg.position.x, target_msg.position.y]):
            self.new_target_flag = 1
	    print("newtarget recieved")
            self.TargetTuple = tuple([target_msg.position.x, target_msg.position.y])
	#print('Self target in func', self.TargetTuple)
	#print('other', self.target_sub.position.x, self.target_sub.position.y)

    def current_pos_interpret(self, pos_msg):
        self.cur_loc = tuple([pos_msg.position.x, pos_msg.position.y])
	#print("pos interpreted!")

    def publish_waypoint(self):
        #print("waypoint published!")
        self.waypoint_pub.publish(self.waypoint_pose)
	#print(self.waypoint_pose)

    def call_a_star(self, start, end):
        return self.fill.flood_fill_do(start, end)



if __name__ == '__main__':
    rospy.init_node('path_manager', log_level=rospy.DEBUG)
    PM = PathManager()
    PM.waypoint_manager()
    #rospy.spin()


