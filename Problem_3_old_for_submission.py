# Lab 4 - A*
# Problem 3 - Flood Fill
# Kyle, Cole, Oliver, Rafael

# importing matplotlib modules, other files, etc.
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
from Problem_1 import PriorityQueue
from Problem_1 import Node
from Problem_2 import graphDataStructure
from matplotlib import colors

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

    def add_to_queue(self, pixel_location, pixel_weight):
        # Adds item to the queue
        # pixel_location = array of i,j location of pixel (i.e. [100, 150]
        # pixel_weight = weight of pixel
        self.priQueue.insert(Node(pixel_location, pixel_weight))


if __name__ == '__main__':
    # Create class instance of the floodFill
    fill = floodFill()
    # Create an matrix to track whether a pixel has been hit
    closed_array = np.zeros((384, 384), dtype=int)
    pixel_loc = [100, 150]

    # Close the pixel we have already seen
    closed_array[pixel_loc[0]][pixel_loc[1]] = 1

    # Create a matrix to track priority values
    imageArray = np.zeros((384, 384), dtype=int)
    imageArray[pixel_loc[0]][pixel_loc[1]] = 0

    # Add the first pixel to the queue
    fill.add_to_queue(pixel_loc, 0)

    # Loop while data is in the priority queue
    while fill.priQueue.size() > 0:
        # for the location array: queue_val[0], for priority: queue_val[1]
        queue_val = fill.priQueue.show()

        # Get the matrix of valid neighboring pixels
        neighbors = fill.dataStructure.checkPixel(queue_val[0])

        imageArray[queue_val[0][0]][queue_val[0][1]] = queue_val[1]
        fill.priQueue.delete()
        # Loop through the neighboring pixels
        for i in range(len(neighbors)):
            if closed_array[neighbors[i][0]][neighbors[i][1]] == 0:

                # Add neighbor to queue with 1 less priority
                fill.add_to_queue([neighbors[i][0], neighbors[i][1]], queue_val[1]-1)
                # Close that one
                closed_array[neighbors[i][0]][neighbors[i][1]] = 1

    # This code normalizes the colors
    abs_vals = np.absolute(imageArray)
    max_val = np.max(abs_vals)
    updated_image = abs_vals/max_val*255
    plt.imshow(np.absolute(updated_image))
    plt.savefig('part3.png', bbox_inches='tight')
    plt.show()



