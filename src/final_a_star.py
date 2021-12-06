#!/usr/bin/env python

# Lab 4 - A*
# Problem 3/4 - Flood Fill
# Kyle, Cole, Oliver, Rafael

# importing matplotlib modules, other files, etc.
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import image as mpimg
import random as r

#import rospy
#from geometry_msgs.msg import Pose

from Problem_2 import graphDataStructure

class Node:
# Priority queue node class
    def __init__(self, info, priority):
        self.info = info
        self.priority = priority

# class for Priority queue
class PriorityQueue:

    def __init__(self):
        self.queue = list()
        # if you want you can set a maximum size for the queue

    def insert(self, node):
        # if queue is empty
        if self.size() == 0:
            # add the new node
            self.queue.append(node)
        else:
            # traverse the queue to find the right place for new node
            for x in range(0, self.size()):
                # if the priority of new node is less
                if node.priority <= self.queue[x].priority:
                    # if we have traversed the complete queue
                    if x == (self.size() - 1):
                        # add new node at the end (most negative)
                        self.queue.insert(x + 1, node)
                    else:
                        continue
                else:
                    self.queue.insert(x, node)

    def delete(self):
        # remove the first node from the queue
        return self.queue.pop(0)

    def show(self):
        return self.queue[0].info, self.queue[0].priority

    def size(self):
        return len(self.queue)

# Data structure code - from part 2 of the lab
class graphDataStructure:
    def __init__(self):
        # Read Images
        # This imports the image
        self.img = mpimg.imread('TymoshuoMap.pgm')
        self.neighbor_pixels = []
        self.border_x = []
        self.border_y = []
        self.pix_neg_y = np.array([])
        self.pix_pos_y = np.array([])
        self.pix_pos_x = np.array([])
        self.pix_neg_x = np.array([])
        self.x_point_select,self.y_point_select = self.find_next_point()
        #print(self.y_point_select)
        #print(self.x_point_select)




    def occupyBuild(self):
        # Create a matrix to store whether position is occupied or not
        self.isOccupied = np.zeros((len(self.img[0]), len(self.img[1])), dtype=int)

        # Loop through each of the pixels. From 0-383 (384 pixels each row/col)
        for x in range(len(self.img[0])):
            for y in range(len(self.img[1])):
                # set all non-yellow spaces as occupied
                if self.img[x, y] != 254:
                    self.isOccupied[x, y] = 1
            # 254 is yellow - open space

    def threshold(self):
        # This will output the threshold image
        #plt.imshow(self.isOccupied)
        plt.savefig('part2.png', bbox_inches='tight')
        plt.imshow(self.img)
        plt.show()


    def nodes(self):
        # This generates the node array and sets it to infinity
        self.nodes = np.full((len(self.img[0]), len(self.img[1])), np.inf)



    def checkPixel(self, pixel):
        # This returns an array of unoccupied points
        # pixel = 2x1 (i, j) of pixel location
        # returns array of i, j  locations

        # Set up array to return
        open_pixels = []

        # Check above location
        if self.isOccupied[int(pixel[0])][int(pixel[1] + 1)] == 0:
            open_pixels.append([int(pixel[0]), int(pixel[1] + 1)])
        # Check above-right location
        if self.isOccupied[int(pixel[0]+1)][int(pixel[1] + 1)] == 0:
            open_pixels.append([int(pixel[0]+1), int(pixel[1] + 1)])
        # Check above-left location
        if self.isOccupied[int(pixel[0]-1)][int(pixel[1] + 1)] == 0:
            open_pixels.append([int(pixel[0]-1), int(pixel[1] + 1)])
        # Check right location
        if self.isOccupied[int(pixel[0] + 1)][int(pixel[1])] == 0:
            open_pixels.append([int(pixel[0] + 1), int(pixel[1])])
        # Check below location
        if self.isOccupied[int(pixel[0])][int(pixel[1] - 1)] == 0:
            open_pixels.append([int(pixel[0]), int(pixel[1] - 1)])
        # Check below-right location
        if self.isOccupied[int(pixel[0]+1)][int(pixel[1] - 1)] == 0:
            open_pixels.append([pixel[0]+1, pixel[1] - 1])
        # Check below-left location
        if self.isOccupied[int(pixel[0]-1)][int(pixel[1] - 1)] == 0:
            open_pixels.append([int(pixel[0]-1), int(pixel[1] - 1)])
        # Check left location
        if self.isOccupied[int(pixel[0] - 1)][int(pixel[1])] == 0:
            open_pixels.append([int(pixel[0] - 1), int(pixel[1])])

        return open_pixels

######### find a new point to explore


    def find_next_point(self):
        pix_num = len(self.img)
        for i in range(pix_num):
            for j in range(pix_num):
                if self.img[i,j] >= 250:
                    val_neg_y = np.append(self.pix_neg_y,self.img[i - 1, j])            # looks in negative 'y' direction of .pgm
                    val_pos_y = np.append(self.pix_pos_y,self.img[i + 1, j])            # Looks for walls in the positive 'y' direction of .pgm
                    val_pos_x = np.append(self.pix_pos_x,self.img[i, j + 1])            # Looks for walls in the positive 'x' direction of .pgm
                    val_neg_x = np.append(self.pix_neg_x,self.img[i, j - 1])            # Looks for walls in the negative 'x' direction of .pgm
                    if val_neg_y == 205:
                        self.border_x = np.append(self.border_x,j)
                        self.border_y = np.append(self.border_y,i)

                    if val_pos_y == 205:
                        self.border_x = np.append(self.border_x, j)
                        self.border_y = np.append(self.border_y, i)

                    if val_neg_x == 205:
                        self.border_x = np.append(self.border_x, j)
                        self.border_y = np.append(self.border_y, i)

                    if val_pos_x == 205:
                        self.border_x = np.append(self.border_x, j)
                        self.border_y = np.append(self.border_y, i)
        plt.scatter(self.border_x, self.border_y, color='black', marker='.')  ### <------------ next spot
        rand_num = r.randrange(0, len(self.border_x))
        for i in range(len(self.border_x)):
            if i == rand_num:
                self.x_point_select = self.border_x[i]
                self.y_point_select = self.border_y[i]



        return self.x_point_select,self.y_point_select









################







class floodFill():
    def __init__(self):
        # Set up priority queue
        self.priQueue = PriorityQueue()

        # Set up graph data structure
        self.dataStructure = graphDataStructure()
        self.dataStructure.occupyBuild()
        self.dataStructure.threshold()
        self.dataStructure.nodes()

        self.x_star = []
        self.y_star = []

    def set_up_structure(self):
        test = graphDataStructure()
        test.occupyBuild()
        test.nodes()

    def add_to_queue(self, pixel_location, pixel_weight, dista):
        # Adds item to the queue
        # pixel_location = array of i,j location of pixel (i.e. [100, 150]
        # pixel_weight = priority value of pixel
        # dista = actual Euclidean distance

        # This inserts into the priority queue
        self.priQueue.insert(Node(np.append(pixel_location,[round(dista,2)]).tolist(), pixel_weight))


    def work_backwards(self, abs_image, end_array, start_array):
        # Works backwards to find the best path
        # inputs: absolute val of priority and start/end pixels
        self.dist_val = abs_image[end_array[0]][end_array[1]]
        self.previous_loc = end_array
        marker_locations = []
        marker_locations.append(end_array)

        best_val = 0
        best_val_loc = 0
        print('new test', self.previous_loc, start_array)
        while self.previous_loc != start_array:
            neighbors = fill.dataStructure.checkPixel(self.previous_loc)
            best_val = abs_image[neighbors[0][0]][neighbors[0][1]]
            best_val_loc = 0
            for i in range(0,(len(neighbors))):
                # Check if better and record neighbor location
                if abs_image[neighbors[i][0]][neighbors[i][1]] < best_val:
                    best_val = abs_image[neighbors[i][0]][neighbors[i][1]]
                    best_val_loc = i
            self.previous_loc = neighbors[best_val_loc]

            # Add the new marker location to the array
            marker_locations.append(self.previous_loc)

        return marker_locations

    def pop_zeros(self,list_items):
        while list_items[-1] == 0:
            list_items.pop()
        return list_items

    def odom_to_map(self, inp):
        new_inp = (inp+10)*(384/20)
        return new_inp

    def flood_fill_do(self, pixel_loc_in, end_pixel_loc_in):
        # Create a matrix to track whether a pixel has been hit
        closed_array = np.zeros((384, 384), dtype=int)
        pixel_loc2 = np.rint(self.odom_to_map(np.asarray(pixel_loc_in)))
        pixel_loc = [0]*2
        pixel_loc[0] = pixel_loc2[0].astype(int)
        pixel_loc[1] = pixel_loc2[1].astype(int)
        end_pixel_loc2 = np.rint(self.odom_to_map(np.asarray(end_pixel_loc_in)))
        end_pixel_loc = [0]*2
        end_pixel_loc[0] = end_pixel_loc2[0].astype(int)
        end_pixel_loc[1] = end_pixel_loc2[1].astype(int)

        # Close the pixel we have already seen
        closed_array[pixel_loc[0]][pixel_loc[1]] = 1

        # Create a matrix to track priority values
        imageArray = np.zeros((384, 384), dtype=int)
        imageArray[pixel_loc[0]][pixel_loc[1]] = 0
        # Add the first pixel to the queue
        self.add_to_queue(pixel_loc, 0, 0)

        # Loop while data is in the priority queue
        while fill.priQueue.size() > 0:
            # for the location array: queue_val[0], for priority: queue_val[1]
            queue_val = self.priQueue.show()
            # Get the matrix of valid neighboring pixels
            neighbors = self.dataStructure.checkPixel(queue_val[0])
            dist = queue_val[0][2]

            imageArray[int(queue_val[0][0])][int(queue_val[0][1])] = queue_val[1]
            self.priQueue.delete()
            # Loop through the neighboring pixels
            for i in range(len(neighbors)):

                if closed_array[int(neighbors[i][0])][int(neighbors[i][1])] < 1:
                    dist = queue_val[0][2] + np.sqrt(
                        (neighbors[i][0] - queue_val[0][0]) ** 2 + (neighbors[i][1] - queue_val[0][1]) ** 2)

                    # Add neighbor to queue with 1 less priority
                    self.add_to_queue([neighbors[i][0], neighbors[i][1]], -dist, dist)

                    # Close that one
                    closed_array[int(neighbors[i][0])][int(neighbors[i][1])] = 1

        # This code normalizes the colors
        abs_vals = np.absolute(imageArray)
        max_val = np.max(abs_vals)
        updated_image = abs_vals / max_val * 255
        plt.imshow(np.absolute(updated_image))

        # Work backwards for path
        #print('test',end_pixel_loc, pixel_loc)
        mark_loc = self.work_backwards(abs_vals, end_pixel_loc, pixel_loc)

        # Get x and y values (note reversed from typical)
        x_vals = [row[0] for row in mark_loc]
        y_vals = [row[1] for row in mark_loc]
        plt.scatter(y_vals[0:], x_vals[0:], color='red', marker='.')


        ######## Make room for wall turns etc
        path_length = range(len(mark_loc))
        count = 0
        prev_count = 0
        pix_expand = 5  # choose distance for pixel separation
        offset = 3  # How far to offset points
        width = 5  # Farthest distance from the wall a pixel can be before it is offset
        pix_val_left = np.array([])
        pix_val_right = np.array([])
        pix_val_up = np.array([])
        pix_val_down = np.array([])

        def check_for_obstical(y, x, width, offset, x_star, y_star):
            for j in range(width):
                val_neg_y = np.append(pix_val_left, updated_image[x - j, y])  # looks in negative 'y' direction of .pgm
                val_pos_y = np.append(pix_val_right,
                                      updated_image[x + j, y])  # Looks for walls in the positive 'y' direction of .pgm
                val_pos_x = np.append(pix_val_up,
                                      updated_image[x, y + j])  # Looks for walls in the positive 'x' direction of .pgm
                val_neg_x = np.append(pix_val_down,
                                      updated_image[x, y - j])  # Looks for walls in the negative 'x' direction of .pgm
                if val_neg_y == 0:
                    x_star.append(x + offset)
                    y_star.append(y)
                if val_pos_y == 0:
                    x_star.append(x - offset)
                    y_star.append(y)
                if val_pos_x == 0:
                    x_star.append(x)
                    y_star.append(y - offset)
                if val_neg_x == 0:
                    x_star.append(x)
                    y_star.append(y + offset)
            return x_star, y_star

        for i in path_length:
            count += 1
            pix_dist = count - prev_count
            if pix_dist >= pix_expand:
                plt.scatter(y_vals[i], x_vals[i], color='black', marker='.')
                prev_count = count
                self.x_star, self.y_star = check_for_obstical(y_vals[i], x_vals[i], width, offset, self.x_star, self.y_star)
        plt.scatter(self.y_star, self.x_star, color='blue', marker='.')
        plt.show()










# def map_odom_xy(data):
#     pose = Pose()
#     try:
#         x = data.position.x
# 	    y = data.position.y
# 	    return (x,y)
#     except AttributeError as e:
# 	    rospy.logwarn(e)
# 	return

# Run the class if main
if __name__ == '__main__':
    def map(x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    #rospy.init_node('map_odom_output_x_y')
    #rospy.Subscriber('/map_odom', Pose, map_odom_xy)
    fill = floodFill()
    #print(tuple([-3, -3]))
    #print(np.asarray(tuple([-3, -3])))

    next_points = graphDataStructure().find_next_point()
    x = next_points[0]
    y = next_points[1]


    new_x = map(x, 0, 384, -10, 10)
    new_y = map(y, 0, 384, -10, 10)
    print(new_x,new_y)



    fill.flood_fill_do(tuple([-3, -1]), tuple([new_x, new_y]))
