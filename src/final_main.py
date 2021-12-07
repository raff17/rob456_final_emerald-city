import final_a_star as a
import rospy


class MainRun:
    def __init__(self):

        self.fill = a.floodFill()
	self.data_struc = a.graphDataStructure()
        #self.next_point = tuple(a.graphDataStructure().find_next_point(self.fill))
	self.next_point = self.data_struc.find_next_point(self.fill)
        self.robot_loc_x =  100 #odometry
        self.robot_loc_y = 75  #odometry
        self.did_send = False #flag_from_ros
	self.target_pub = rospy.Publisher('/Current_Target', String, queue_size=10)


    def main_run_func(self,time_limit,beg_time,current_time):
	print(self.next_point)
        current_time = rospy.get_rostime()
        limit = 10
        x = self.next_point[0]
        y = self.next_point[1]
        diff_x = abs(x-self.robot_loc_x)
        diff_y = abs(y - self.robot_loc_y)

        if self.did_send == True:
            beg_time = rospy.get_rostime()     # at this point the robot should begin it's move.
            self.did_send = False


        time_taken = current_time - beg_time
        #print(time_taken)
        if time_taken > time_limit:

            if diff_x > limit and diff_y > limit:
                print("robot didn't reach point")
                beg_time = current_time
                self.target_pub.publish(String(str(self.next_point)))
        	#print("target published!")  # This should change based on the new explored map

        elif diff_x <= limit and diff_y <= limit:
            beg_time = current_time
            self.target_pub.publish(String(str(self.next_point)))
  	    #print("target published!")

    def publish_target(self):
        #print("target published!")
        self.target_pub.publish(String(str(self.next_point)))


if __name__ == '__main__':
    time_limit = 2 # Time allowed for robot to reach destination in ms?
    beg_time = 0
    current_time = 0
    rospy.init_node('final_main')
    rospy.spin()
    #rate = rospy.Rate(1)
    #rate.sleep()
    Planning = MainRun()
    Planning.main_run_func()
    #print(newpoint.is_bot_at_endpoint(time_limit,beg_time,current_time))
