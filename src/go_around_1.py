#!/usr/bin/env python

import rospy
import numpy as np

# Sensor message types
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# the velocity command message
from geometry_msgs.msg import Twist

# String command message
from std_msgs.msg import String

ODOM = None


def lidar_callback(scan_msg):
    global GOAL, ODOM
    # Let's make a new twist message
    command = Twist()

    # Fill in the fields.  Field values are unspecified 
    # until they are actually assigned. The Twist message 
    # holds linear and angular velocities.
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

    # Lidar properties (unpacked for your ease of use)
    # find current laser angle, max scan length, distance array for all scans, and number of laser scans
    maxAngle = scan_msg.angle_max
    minAngle = scan_msg.angle_min
    angleIncrement = scan_msg.angle_increment

    maxScanLength = scan_msg.range_max
    distances = scan_msg.ranges
    numScans = len(distances)

    print(ODOM)

    # Problem 1: move the robot toward the goal
    # create vector between goal and cur. pos.
    vect = [GOAL[0] - ODOM[0], GOAL[1] - ODOM[1]]
    # create angle between goal and cur. pos.
    rot_goal_angle = np.arctan2(vect[1], vect[0])

    # rotate robot
    rot_vel = 0.35
    lin_vel = 0.16
    angular_var = 0

    # normal goal finding 
    if abs(rot_goal_angle - ODOM[2]) > 0.02:
        if rot_goal_angle - ODOM[2] > 0:
            angular_var = rot_vel
            # command.angular.z = rot_vel
        elif rot_goal_angle - ODOM[2] < 0:
            # command.angular.z = -rot_vel
            angular_var = -rot_vel
        else:
            angular_var = 0

    if abs(vect[0] - ODOM[0]) or abs(vect[1] - ODOM[1]) > 0.5:
        command.linear.x = lin_vel

    # YOUR CODE HERE
    # End problem 1

    RstopTrigger = 0
    RnumTriggerscans = 0
    LstopTrigger = 0
    LnumTriggerscans = 0
    backupTrigger = 0
    dist_trig = 0.6
    left_ang_bound = 2 * np.pi
    right_ang_bound = 0
    scale = 1.4

    currentLaserTheta = minAngle
    # for each laser scan
    for i, scan in enumerate(distances):
        # for each laser scan, the angle is currentLaserTheta, the index is i, and the distance is scan
        # Problem 2: avoid obstacles based on laser scan readings
        # TODO YOUR CODE HERE

        # only take scans from in front of robot
        if right_ang_bound < currentLaserTheta < 0.6 or 5.65 < currentLaserTheta < left_ang_bound:
            # figure out which side of robot object is on
            if 0 < currentLaserTheta < 1.5:
                # thus object is on right of the robot
                # find if it is close to the bot
                if scan < dist_trig:
                    RstopTrigger += 1
                RnumTriggerscans += 1


            # other case is currentLaserTheta - minAngle < 0
            else:
                # object is on left of robot
                # find if it is close to the bot
                if scan < dist_trig:
                    LstopTrigger += 1
                LnumTriggerscans += 1

        if RnumTriggerscans > 0:
            # take average of scans to ensure object is at the right    
            if float(RstopTrigger) / float(RnumTriggerscans) > 0.8:
                # set var for only doing corrective action
                # stop the bot, rotate to the left
                command.linear.x = 0
                # command.angular.z = rot_vel*3
                if angular_var > 0:
                    # increase speed to left
                    angular_var = angular_var * 1 * scale
                else:
                    # change direction to rotate to the left
                    angular_var = angular_var * -1 * scale

        if LnumTriggerscans > 0:
            if float(LstopTrigger) / float(LnumTriggerscans) > 0.8:
                # set var for only doing corrective action
                # stop the bot, rotate to the right
                command.linear.x = 0
                # command.angular.z = -rot_vel*3
                if angular_var < 0:
                    # increase rot speed to right
                    angular_var = angular_var * (1) * scale
                else:
                    # change direction to rotate to the right
                    angular_var = angular_var * (-1) * scale

            if float(LstopTrigger) / float(LnumTriggerscans) > 0.8 and float(RstopTrigger) / float(
                    RnumTriggerscans) > 0.8:
                command.linear.x = -1

        # End problem 2

        # After this loop is done, we increment the currentLaserTheta
        currentLaserTheta = currentLaserTheta + angleIncrement

    command.angular.z = angular_var
    pub.publish(command)


def odom_callback(msg):
    """
    Subscribes to the odom message, unpacks and transforms the relevent information, and places it in the global variable ODOM
    ODOM is structured as follows:
    ODOM = (x, y, yaw)

    :param: msg: Odometry message
    :returns: None
    """
    global ODOM
    position = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    (r, p, yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    ODOM = (position.x, position.y, yaw)


def target_callback(target_msg):  # Check if correct plz ;-;
    """
    Sets go_around as a subscriber to the tuples of the wave-point
    :param target_msg:
    :return: None
    """
    global GOAL
    GOAL = tuple(target_msg)


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('lab2', log_level=rospy.DEBUG)

    # subscribe to sensor messages
    lidar_sub = rospy.Subscriber('/scan', LaserScan, lidar_callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    waypoint_sub = rospy.Subscriber('/Current_Waypoint', string, target_callback)
    # publish twist message
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Turn control over to ROS
    rospy.spin()
