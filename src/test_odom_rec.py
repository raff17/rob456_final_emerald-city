#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose


def map_odom_xy(data):
    pose = Pose()
    try:
        x = data.position.x
	y = data.position.y
        print(x)
	return (x,y)
    except AttributeError as e:
	rospy.logwarn(e)
	return


if __name__ == '__main__':
    rospy.init_node('map_odom_output_x_y')
    rospy.Subscriber('/map_odom', Pose, map_odom_xy)
    
    while not Pose:
	rospy.spin()


