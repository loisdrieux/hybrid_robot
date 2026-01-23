#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time

# ROS 2 message imports
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

# Corrected import: Remove the dot for ROS 2 standalone execution
import rrt 
from rrt import RRT, collision

class RRTPlannerNode(Node):
    def __init__(self):
        super().__init__('rrt_planner_node')
        
        # --- Parameters ---
        self.step_size = 0.1  # Planning step in meters
        self.map_data = None  # To store the received OccupancyGrid
        self.latest_path = None  # To store the path for persistent publishing
        
        # --- Subscribers ---
        # Get map data from the map_server
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            10)
        
        # --- Publishers ---
        # Output the path to be visualized in RViz
        self.path_pub = self.create_publisher(Path, '/rrt_path', 10)
        
        # --- Timer ---
        # Periodically republish the path to ensure RViz receives it
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("RRT* Node started. Persistent publishing enabled.")

    def timer_callback(self):
        """ Periodically publishes the saved path. """
        if self.latest_path is not None:
            self.publish_path(self.latest_path)

    def map_callback(self, msg):
        """ Stores map data and triggers simulation once. """
        if self.map_data is None:
            self.map_data = msg
            self.get_logger().info("Map received! Running RRT planning...")
            self.run_simulation()

    def run_simulation(self):
        """ Main logic to call the RRT algorithm. """
        
        rand_area = [0.1, 9.8, 0.1, 9.9] 
    
        # Use the start/goal that correspond to the visual map
        start = [1.5, 5.0]  
        goal = [9.5, 5.0]
        self.get_logger().info(f"Planning from {start} to {goal}...")
        start_time = time.time()

        # Initialize the RRT algorithm class
        planner = RRT(
            start=start,
            goal=goal,
            rand_area=rand_area,
            step_size=self.step_size,
            map_data=self.map_data
        )
        
        # Compute the path
        path = planner.planning()
        
        end_time = time.time()

        if path:
            self.get_logger().info(f"Path found in {end_time - start_time:.4f}s")
            self.latest_path = path  # Save the path to be republished by the timer
        else:
            self.get_logger().warn("RRT failed to find a valid path.")

    def publish_path(self, rrt_path):
        """ Converts point list to nav_msgs/Path and publishes. """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for point in rrt_path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            # Set Z to 0.1 to ensure visibility above the floor in RViz
            pose.pose.position.z = 0.1 
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RRTPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()