# Lab 4 - A*
# Problem 2 - Graph Data Structure
# Kyle, Cole, Olver, Rafael

# importing matplotlib modules
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np


class graphDataStructure:
    def __init__(self):
        # Read Images
        # This imports the image
        self.img = mpimg.imread('dufrenek.pgm')
        self.neighbor_pixels = []

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
        plt.imshow(self.isOccupied)
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


# Run the class if main
if __name__ == '__main__':
    test = graphDataStructure()
    test.occupyBuild()
    test.threshold()
    test.nodes()
    neighbor_pixels = test.checkPixel([100, 150])
    # Output Images
    plt.imshow(test.img)
    plt.show()
