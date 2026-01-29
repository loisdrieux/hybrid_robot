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
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent_x = []
        self.parent_y = []
        self.parent_z = []

# check collision
import numpy as np

def collision_3d(x1, y1, z1, x2, y2, z2, map_data):
    """
    Checks for collisions. If altitude Z > 1.0m, we assume we fly over 
    the shelves (2D obstacles).
    """
    # If starting and finishing point are in altitude
    if z1 > 1.0 and z2 > 1.0:
        return False 

    # Sinon, on vérifie la collision standard sur la map 2D
    dist = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    theta = np.arctan2(y2 - y1, x2 - x1)
    res = map_data.info.resolution
    test_steps = int(dist / res)
    
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y
    width = map_data.info.width
    height = map_data.info.height
    safety_margin_px = 3

    for i in range(test_steps):
        curr_x = x1 + i * (res / 2.0) * np.cos(theta)
        curr_y = y1 + i * (res / 2.0) * np.sin(theta)
        gx = int((curr_x - origin_x) / res)
        gy = int((curr_y - origin_y) / res)
        
        for dx in range(-safety_margin_px, safety_margin_px + 1):
            for dy in range(-safety_margin_px, safety_margin_px + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    index = ny * width + nx
                    if map_data.data[index] > 50 or map_data.data[index] == -1:
                        return True
    return False
    
# check the  collision with obstacle
def check_collision_3d(x1, y1, z1, x2, y2, z2, map_data, goal, step_size):
    """
    Calculates the new point position in 3D and checks for obstacles.
    """
    # Calcultaion
    dx = x1 - x2
    dy = y1 - y2
    dz = z1 - z2
    dist_total = math.sqrt(dx**2 + dy**2 + dz**2)
    
    if dist_total == 0:
        return x2, y2, z2, False, False

    # Step size
    tx = x2 + (dx / dist_total) * step_size
    ty = y2 + (dy / dist_total) * step_size
    tz = z2 + (dz / dist_total) * step_size

    # Limitations of the map
    res = map_data.info.resolution
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y
    grid_x = int((tx - origin_x) / res)
    grid_y = int((ty - origin_y) / res)

    if grid_x < 0 or grid_x >= map_data.info.width or grid_y < 0 or grid_y >= map_data.info.height:
        return tx, ty, tz, False, False

    # Check collision
    nodeCon = not collision_3d(x2, y2, z2, tx, ty, tz, map_data)
    # Check if we can access the goal from this point
    directCon = not collision_3d(tx, ty, tz, goal[0], goal[1], goal[2], map_data)

    return tx, ty, tz, directCon, nodeCon


Z_PENALTY = 5.0 #To stay on the ground in the majority 
class RRT:
    def __init__(self, start, goal, rand_area, step_size, map_data):
        self.start = start 
        self.goal = goal 
        self.rand_area = rand_area 
        self.step_size = step_size
        self.map_data = map_data
        self.node_list = [Nodes(start[0], start[1], start[2])]

        
    def dist_and_angle_3d(self,x1, y1, z1, x2, y2, z2):
        dist = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
        theta = math.atan2(y2-y1, x2-x1) 
        return dist, theta

    def get_nearest_node_index(self, nx, ny, nz):
        """
        Finds the nearest node in 3D, applying a penalty to Z to favor ground nodes.
        """
        
        dlist = [
            (node.x - nx)**2 + (node.y - ny)**2 + ((node.z - nz)**2 * Z_PENALTY) 
            for node in self.node_list
        ]
        return dlist.index(min(dlist))

    self.collision_count = 0
    self.total_samples = 0
    def planning(self):
        for i in range(5000):
            # 70% of the time, try to stay on the ground
            # This forces the robot to look for a 2D path first
            self.total_samples += 1
            if random.random() < 0.7:
                nz = 0.0
            else:
                nz = random.uniform(self.rand_area[4], self.rand_area[5])
                
            nx = random.uniform(self.rand_area[0], self.rand_area[1])
            ny = random.uniform(self.rand_area[2], self.rand_area[3])
            
            dlist = [
                (n.x - nx)**2 + (n.y - ny)**2 + ((n.z - nz)**2 * Z_PENALTY) 
                for n in self.node_list
            ]
            
            nearest_ind = dlist.index(min(dlist))
            near_node = self.node_list[nearest_ind]

            # Direction vector
            dist_xy, theta_xy = self.dist_and_angle_3d(nx, ny, nz, near_node.x, near_node.y, near_node.z)

            # English comment: Calculate vertical angle (phi) using the altitude difference
            phi = math.atan2(nz - near_node.z, dist_xy)

            # English comment: 3D Projection for the new point (tx, ty, tz)
            tx = near_node.x + self.step_size * math.cos(phi) * math.cos(theta_xy)
            ty = near_node.y + self.step_size * math.cos(phi) * math.sin(theta_xy)
            tz = near_node.z + self.step_size * math.sin(phi)


            if not collision_3d(near_node.x, near_node.y, near_node.z, tx, ty, tz, self.map_data):
                new_node = Nodes(tx, ty, tz)
                new_node.parent_x = near_node.parent_x + [tx]
                new_node.parent_y = near_node.parent_y + [ty]
                new_node.parent_z = near_node.parent_z + [tz]
                self.node_list.append(new_node)

                # Check if goal reached
                d_goal = math.sqrt((tx-self.goal[0])**2 + (ty-self.goal[1])**2 + (tz-self.goal[2])**2)
                if d_goal < self.step_size:
                    path = [[px, py, pz] for px, py, pz in zip(new_node.parent_x, new_node.parent_y, new_node.parent_z)]
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
