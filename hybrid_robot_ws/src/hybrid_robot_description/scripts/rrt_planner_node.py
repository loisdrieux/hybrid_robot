#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time

# ROS 2 message imports
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker #To display a marker point on RVIZ


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
        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)

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
        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)
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
            self.get_logger().info("Map received! Running RRT planning...")
            time.sleep(3.0) #Wait for the map to load completely
            self.map_data = msg
            self.get_logger().info("Running RRT planning now...")
            self.run_simulation()

    def run_simulation(self):
        """ Main logic to call the RRT algorithm. """
        
        rand_area = [0.1, 9.8, 0.1, 9.9] 
    
        # Use the start/goal that correspond to the visual map
        start = [0.5, 5.0]  
        goal = [9.5, 7.5 ]   #Behind the second obstacle
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

    def publish_goal_marker(self, goal):
        """
        Publishes a red sphere in RViz to represent the goal point.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Goal coordinates
        marker.pose.position.x = float(goal[0])
        marker.pose.position.y = float(goal[1])
        marker.pose.position.z = 0.5 # Slightly above the floor
        
        # Size of the sphere (0.3m)
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # Color: Bright Red
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0 # Fully opaque
        
        self.marker_pub.publish(marker)

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