import a_star as a
import rospy





class NewPoint:
    def __init__(self):

        self.fill = a.floodFill()
        self.next_point = a.graphDataStructure().find_next_point(self.fill)
        self.robot_loc_x =  100 #odometry
        self.robot_loc_y = 75  #odometry
        self.did_send = False #flag_from_ros




    def is_bot_at_endpoint(self,time_limit,beg_time,current_time):
        current_time = rospy.get_rostime()
        limit = 10
        x = self.next_point[0]
        y = self.next_point[1]
        diff_x = abs(x-self.robot_loc_x)
        diff_y = abs(x - self.robot_loc_y)

        if self.did_send == True:
            beg_time = rospy.get_rostime()     # at this point the robot should begin it's move.
            self.did_send = False


        time_taken = current_time - beg_time
        print(time_taken)
        if time_taken > time_limit:

            if diff_x > limit and diff_y > limit:
                print("robot didn't reach point")
                beg_time = current_time
                return self.next_point     # This should change based on the new explored map
        else:
            beg_time = current_time
            return self.next_point







if __name__ == '__main__':
    time_limit = 2 # Time allowed for robot to reach destination in ms?
    beg_time = 0
    current_time = 0
    rospy.init_node('final_main')
    rospy.spin()
    rate = rospy.rate(1)
    rate.sleep()
    newpoint = NewPoint()
    print(newpoint.is_bot_at_endpoint(time_limit,beg_time,current_time))
