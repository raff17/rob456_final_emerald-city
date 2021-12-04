# Lab 4 - A*
# Problem 3/4 - Flood Fill
# Kyle, Cole, Oliver, Rafael

# importing matplotlib modules, other files, etc.
import matplotlib.pyplot as plt
import numpy as np
from Problem_1 import PriorityQueue
from Problem_1 import Node
from Problem_2 import graphDataStructure

class floodFill:
    def __init__(self):
        # Set up priority queue
        self.priQueue = PriorityQueue()

        # Set up graph data structure
        self.dataStructure = graphDataStructure()
        self.dataStructure.occupyBuild()
        self.dataStructure.threshold()
        self.dataStructure.nodes()

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



if __name__ == '__main__':
    #rospy.init_node('map_odom_output_x_y')
    #rospy.Subscriber('/map_odom', Pose, map_odom_callback)
    # Create class instance of the floodFill
    fill = floodFill()
    # Create an matrix to track whether a pixel has been hit
    closed_array = np.zeros((384, 384), dtype=int)
    pixel_loc = [230, 75]
    end_pixel_loc = [100, 340]
    # Close the pixel we have already seen
    closed_array[pixel_loc[0]][pixel_loc[1]] = 1


    # Create a matrix to track priority values
    imageArray = np.zeros((384, 384), dtype=int)
    imageArray[pixel_loc[0]][pixel_loc[1]] = 0
    # Add the first pixel to the queue
    fill.add_to_queue(pixel_loc, 0, 0)

    # Loop while data is in the priority queue
    while fill.priQueue.size() > 0:
        # for the location array: queue_val[0], for priority: queue_val[1]
        queue_val = fill.priQueue.show()
        # Get the matrix of valid neighboring pixels
        neighbors = fill.dataStructure.checkPixel(queue_val[0])
        dist = queue_val[0][2]

        imageArray[int(queue_val[0][0])][int(queue_val[0][1])] = queue_val[1]
        fill.priQueue.delete()
        # Loop through the neighboring pixels
        for i in range(len(neighbors)):

            if closed_array[int(neighbors[i][0])][int(neighbors[i][1])] < 1:
                dist = queue_val[0][2]+np.sqrt((neighbors[i][0]-queue_val[0][0])**2+(neighbors[i][1]-queue_val[0][1])**2)
                term1 = (neighbors[i][0]-queue_val[0][0])**2
                term2 = (neighbors[i][1]-queue_val[0][1])**2
                dist
                # Add neighbor to queue with 1 less priority
                fill.add_to_queue([neighbors[i][0], neighbors[i][1]], -dist, dist)
                test = fill.priQueue.show()
                # Close that one
                closed_array[int(neighbors[i][0])][int(neighbors[i][1])] = 1

    # This code normalizes the colors
    abs_vals = np.absolute(imageArray)
    max_val = np.max(abs_vals)
    updated_image = abs_vals/max_val*255
    plt.imshow(np.absolute(updated_image))

    # Work backwards for path
    mark_loc = fill.work_backwards(abs_vals, end_pixel_loc, pixel_loc)

    # Get x and y values (note reversed from typical)
    x_vals = [row[0] for row in mark_loc]
    y_vals = [row[1] for row in mark_loc]
    plt.scatter(y_vals[0:], x_vals[0:], color='red', marker='.')
    plt.show()


