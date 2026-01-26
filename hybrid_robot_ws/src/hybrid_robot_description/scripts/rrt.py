"""

Path planning with Rapidly-Exploring Random Trees (RRT)

author: Aakash(@nimrobotics)
web: nimrobotics.github.io

"""

import cv2
import numpy as np
import math
import random
import argparse
import os

class Nodes:
    """Class to store the RRT graph"""
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.parent_x = []
        self.parent_y = []

# check collision
import numpy as np

def collision(x1, y1, x2, y2, map_data):
    """
    Checks for collisions between two points (x1, y1) and (x2, y2).
    Includes a safety buffer (inflation) around obstacles.
    """
    dist = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    theta = np.arctan2(y2 - y1, x2 - x1)
    
    res = map_data.info.resolution
    # Increase precision: check every 2.5cm instead of 5cm
    test_steps = int(dist / (res / 2.0))
    
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y
    width = map_data.info.width
    height = map_data.info.height

    for i in range(test_steps):
        curr_x = x1 + i * (res / 2.0) * np.cos(theta)
        curr_y = y1 + i * (res / 2.0) * np.sin(theta)
        
        # Convert world coordinates to grid index
        gx = int((curr_x - origin_x) / res)
        gy = int((curr_y - origin_y) / res)
        
        # Check current pixel and neighbors (Inflation)
        # This adds a small safety margin so the robot doesn't scratch the walls
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    index = ny * width + nx
                    # Obstacle detected if value > 50 or unknown (-1)
                    if map_data.data[index] > 50 or map_data.data[index] == -1:
                        return True
    return False
    
# check the  collision with obstacle and trim
def check_collision(x1, y1, x2, y2, map_data, end_goal, step_size): # Add step_size here
    """
    Validates a new node and its connections.
    """
    _, theta = dist_and_angle(x2, y2, x1, y1)
    
    # Use the step_size passed as argument
    x = x2 + step_size * np.cos(theta)
    y = y2 + step_size * np.sin(theta)

    # ... rest of the function remains the same ...
    res = map_data.info.resolution
    width = map_data.info.width
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y

    grid_x = int((x - origin_x) / res)
    grid_y = int((y - origin_y) / res)

    if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= map_data.info.height:
        return (x, y, False, False)

    directCon = not collision(x, y, end_goal[0], end_goal[1], map_data)
    nodeCon = not collision(x, y, x2, y2, map_data)

    return (x, y, directCon, nodeCon)


# return dist and angle b/w new point and nearest node
def dist_and_angle(x1,y1,x2,y2):
    dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
    angle = math.atan2(y2-y1, x2-x1)
    return(dist,angle)

# return the neaerst node index
def nearest_node(x,y):
    temp_dist=[]
    for i in range(len(node_list)):
        dist,_ = dist_and_angle(x,y,node_list[i].x,node_list[i].y)
        temp_dist.append(dist)
    return temp_dist.index(min(temp_dist))

# generate a random point in the image space
def rnd_point(h,l):
    new_y = random.randint(0, h)
    new_x = random.randint(0, l)
    return (new_x,new_y)


class RRT:
    def __init__(self, start, goal, rand_area, step_size, map_data):
        self.start = start
        self.goal = goal
        self.rand_area = rand_area # [min_x, max_x, min_y, max_y]
        self.step_size = step_size
        self.map_data = map_data
        # Initialize node list with the start position
        self.node_list = [Nodes(start[0], start[1])]
        self.node_list[0].parent_x = [start[0]]
        self.node_list[0].parent_y = [start[1]]

    def planning(self):
        # Implementation of the RRT loop using meters
        for i in range(5000): # max iterations
            # Sample a random point within the terrestrial bounds
            nx = random.uniform(self.rand_area[0], self.rand_area[1])
            ny = random.uniform(self.rand_area[2], self.rand_area[3])

            # Find nearest node
            dlist = [(node.x - nx)**2 + (node.y - ny)**2 for node in self.node_list]
            nearest_ind = dlist.index(min(dlist))
            nearest_node = self.node_list[nearest_ind]

            # Use the check_collision logic we integrated earlier
            tx, ty, directCon, nodeCon = check_collision( nx, ny, nearest_node.x, nearest_node.y, self.map_data, self.goal, self.step_size)

            if nodeCon:
                new_node = Nodes(tx, ty)
                new_node.parent_x = nearest_node.parent_x + [tx]
                new_node.parent_y = nearest_node.parent_y + [ty]
                self.node_list.append(new_node)

                if directCon:
                    # Target reached: return the path as a list of [x, y]
                    path = [[px, py] for px, py in zip(new_node.parent_x, new_node.parent_y)]
                    path.append([self.goal[0], self.goal[1]])
                    return path
        return None

    def get_nearest_node_index(self, x, y):
        dlist = [(node.x - x)**2 + (node.y - y)**2 for node in self.node_list]
        return dlist.index(min(dlist))


def draw_circle(event,x,y,flags,param):
    global coordinates
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img2,(x,y),5,(255,0,0),-1)
        coordinates.append(x)
        coordinates.append(y)


def RRTstar(img, img2, start, end, stepSize):
    """
    add rrt* implementation
    """
    pass



if __name__ == '__main__':

    parser = argparse.ArgumentParser(description = 'Below are the params:')
    parser.add_argument('-p', type=str, default='world2.png',metavar='ImagePath', action='store', dest='imagePath',
                    help='Path of the image containing mazes')
    parser.add_argument('-s', type=int, default=10,metavar='Stepsize', action='store', dest='stepSize',
                    help='Step-size to be used for RRT branches')
    parser.add_argument('-start', type=int, default=[20,20], metavar='startCoord', dest='start', nargs='+',
                    help='Starting position in the maze')
    parser.add_argument('-stop', type=int, default=[450,250], metavar='stopCoord', dest='stop', nargs='+',
                    help='End position in the maze')
    parser.add_argument('-selectPoint', help='Select start and end points from figure', action='store_true')

    args = parser.parse_args()

    # remove previously stored data
    try:
      os.system("rm -rf media")
    except:
      print("Dir already clean")
    os.mkdir("media")

    img = cv2.imread(args.imagePath,0) # load grayscale maze image
    img2 = cv2.imread(args.imagePath) # load colored maze image
    start = tuple(args.start) #(20,20) # starting coordinate
    end = tuple(args.stop) #(450,250) # target coordinate
    stepSize = args.stepSize # stepsize for RRT
    node_list = [0] # list to store all the node points

    coordinates=[]
    if args.selectPoint:
        print("Select start and end points by double clicking, press 'escape' to exit")
        cv2.namedWindow('image')
        cv2.setMouseCallback('image',draw_circle)
        while(1):
            cv2.imshow('image',img2)
            k = cv2.waitKey(20) & 0xFF
            if k == 27:
                break
        # print(coordinates)
        start=(coordinates[0],coordinates[1])
        end=(coordinates[2],coordinates[3])

    # run the RRT algorithm 
    RRT(img, img2, start, end, stepSize)
