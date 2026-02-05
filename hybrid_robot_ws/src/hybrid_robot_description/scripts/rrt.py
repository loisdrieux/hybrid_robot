"""
Path planning with Rapidly-Exploring Random Trees (RRT) en 3D
"""

import cv2
import numpy as np
import math
import random
import argparse
import os

class Nodes:
    """Class to store the RRT graph"""
    # 2D Version: def __init__(self, x, y):
    def __init__(self, x, y, z): # 3D Version
        self.x = x
        self.y = y
        self.z = z 
        self.parent_x = []
        self.parent_y = []
        self.parent_z = [] 

# check collision
import numpy as np

# 2D Version: def collision(x1, y1, x2, y2, map_data):
def collision(x1, y1, z1, x2, y2, z2, map_data): # 3D Version
    """
    Checks for collisions between two points in 3D.
    Si Z > hauteur_obstacle (1.5m), pas de collision.
    """
    # Height of the obstacles
    obstacle_height_limit = 2.2 #2m obstacles + size of the robot approximately
    
    # 2D Version: dist = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    dist = np.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2) # 3D distance
    theta = np.arctan2(y2 - y1, x2 - x1)
    # pitch
    phi = np.arctan2(z2 - z1, np.sqrt((x2 - x1)**2 + (y2 - y1)**2))
    
    res = map_data.info.resolution
    test_steps = int(dist / (res / 2.0))
    
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y
    width = map_data.info.width
    height = map_data.info.height
    safety_margin_px = 10

    for i in range(test_steps):
        curr_x = x1 + i * (res / 2.0) * np.cos(theta) * np.cos(phi)
        curr_y = y1 + i * (res / 2.0) * np.sin(theta) * np.cos(phi)
        curr_z = z1 + i * (res / 2.0) * np.sin(phi)
        
        # If we are higher than 2m, there are no obstacle
        if curr_z > obstacle_height_limit:
            continue

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
    
# 2D Version: def check_collision(x1, y1, x2, y2, map_data, end_goal, step_size):
def check_collision(x1, y1, z1, x2, y2, z2, map_data, end_goal, step_size): # 3D Version
    """
    Validates a new node and its connections in 3D.
    """
    # 2D Version: dist, theta = dist_and_angle(x2, y2, x1, y1)
    dist, theta, phi = dist_and_angle_3d(x2, y2, z2, x1, y1, z1) # 3D Version
    
    x = x2 + step_size * np.cos(theta) * np.cos(phi)
    y = y2 + step_size * np.sin(theta) * np.cos(phi)
    z = z2 + step_size * np.sin(phi)

    res = map_data.info.resolution
    width = map_data.info.width
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y

    grid_x = int((x - origin_x) / res)
    grid_y = int((y - origin_y) / res)

    if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= map_data.info.height or z < 0 or z > 2.0:
        return (x, y, z, False, False)

    # 2D Check: directCon = not collision(x, y, end_goal[0], end_goal[1], map_data)
    directCon = not collision(x, y, z, end_goal[0], end_goal[1], end_goal[2], map_data)
    nodeCon = not collision(x, y, z, x2, y2, z2, map_data)

    return (x, y, z, directCon, nodeCon)


# 2D Version: def dist_and_angle(x1,y1,x2,y2):
def dist_and_angle_3d(x1, y1, z1, x2, y2, z2): # 3D Version
    dist = math.sqrt(((x1-x2)**2) + ((y1-y2)**2) + ((z1-z2)**2))
    theta = math.atan2(y2-y1, x2-x1) # Angle XY (Yaw)
    phi = math.atan2(z2-z1, math.sqrt((x2-x1)**2 + (y2-y1)**2)) # Angle Z (Pitch)
    return (dist, theta, phi)


class RRT:
    def __init__(self, start, goal, rand_area, step_size, map_data,z_penalty):
        self.start = start 
        self.goal = goal   
        self.rand_area = rand_area  
        self.step_size = step_size
        self.map_data = map_data
        self.z_penalty = z_penalty
        
        # Initialize node list with the start position (3D)
        self.node_list = [Nodes(start[0], start[1], start[2])]
        self.node_list[0].parent_x = [start[0]]
        self.node_list[0].parent_y = [start[1]]
        self.node_list[0].parent_z = [start[2]]

    def planning(self):
        for i in range(30000): 
            # 2D: nx = random.uniform(self.rand_area[0], self.rand_area[1])
            # 70% of the time, force the random point to be on the ground (Z=0)
            if random.random() < 0.7:
                nx = random.uniform(self.rand_area[0], self.rand_area[1])
                ny = random.uniform(self.rand_area[2], self.rand_area[3])
                nz = 0.0
            else:
                # 30% of the time, explore the 3D
                nx = random.uniform(self.rand_area[0], self.rand_area[1])
                ny = random.uniform(self.rand_area[2], self.rand_area[3])
                nz = random.uniform(self.rand_area[4], self.rand_area[5])

            # Without penalty 3D
            #nx = random.uniform(self.rand_area[0], self.rand_area[1])
            #ny = random.uniform(self.rand_area[2], self.rand_area[3])
            #nz = random.uniform(self.rand_area[4], self.rand_area[5])

            # 2D Find nearest node: dlist = [(node.x - nx)**2 + (node.y - ny)**2 for node in self.node_list]
            dlist = []
            for node in self.node_list:
                dx = node.x - nx
                dy = node.y - ny
                dz = (node.z - nz) * self.z_penalty
                dist = dx**2 + dy**2 + dz**2 
                dlist.append(dist)

            nearest_ind = dlist.index(min(dlist))
            nearest_node = self.node_list[nearest_ind]

            # 3D Check Collision
            tx, ty, tz, directCon, nodeCon = check_collision(nx, ny, nz, nearest_node.x, nearest_node.y, nearest_node.z, self.map_data, self.goal, self.step_size)

            if nodeCon:
                new_node = Nodes(tx, ty, tz)
                new_node.parent_x = nearest_node.parent_x + [tx]
                new_node.parent_y = nearest_node.parent_y + [ty]
                new_node.parent_z = nearest_node.parent_z + [tz]
                self.node_list.append(new_node)

                if directCon:
                    # Target reached: return path as [x, y, z]
                    path = [[px, py, pz] for px, py, pz in zip(new_node.parent_x, new_node.parent_y, new_node.parent_z)]
                    path.append([self.goal[0], self.goal[1], self.goal[2]])
                    return path
        return None