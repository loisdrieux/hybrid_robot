#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time

# ROS 2 message imports
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker #To display a marker point on RVIZ
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Empty

# Corrected import: Remove the dot for ROS 2 standalone execution
import rrt 
from rrt import RRT, collision

import math
from nav_msgs.msg import Odometry

class RRTPlannerNode(Node):
    def __init__(self):
        super().__init__('rrt_planner_node')
        

        self.robot_started = False
        self.planning_timer = None 
        self.get_logger().info("System ready. Waiting for map, then use '/start_robot' to move.")
        
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        tree_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL # Keeps the tree in memory for RViz
        )

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL 
        )

        # Parameters
        self.step_size = 0.5  # Planning step in meters
        self.control_timer = None 
        self.map_data = None
        self.current_path = [] 
        self.latest_path = None 
        self.latest_goal = None
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # Movement Constraints
        self.target_reached_dist = 0.25
        self.linear_speed = 0.15
        self.angular_speed = 0.4

        # Timer 
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.marker_timer = self.create_timer(1.0, self.timer_callback)

        # Subscribers
        self.start_sub = self.create_subscription(Empty, '/start_robot', self.start_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos) 
        self.odom_sub = self.create_subscription(Odometry, 'odom_drone', self.odom_callback, 10)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/rrt_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_drone', 10)
        self.marker_pub = self.create_publisher(Marker, 'goal_marker', marker_qos)
        self.tree_pub = self.create_publisher(Marker, '/rrt_tree', tree_qos)
        
        self.get_logger().info("RRT* Node started. Persistent publishing enabled.")


    def start_callback(self, msg):
        self.get_logger().info("Starting signal received, the robot will start moving")
        self.robot_started = True

    def timer_callback(self):
        """ Periodically publishes the saved path. """
        if self.latest_path is not None:
            self.publish_path(self.latest_path) #Path
        if self.latest_goal is not None:
            self.publish_goal_marker(self.latest_goal) #Goal marker

    def follow_path(self):
        if not self.robot_started or not self.current_path:
            self.stop_robot()
            return

        target_x, target_y = self.current_path[0]
    
        robot_map_x = self.robot_pose['x']
        robot_map_y = self.robot_pose['y']

        #Distance
        dx = target_x - robot_map_x
        dy = target_y - robot_map_y
        dist = math.sqrt(dx**2 + dy**2)
    
        # Angle
        angle_to_target = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(angle_to_target - self.robot_pose['yaw']), 
                        math.cos(angle_to_target - self.robot_pose['yaw']))

        # Debug
        self.get_logger().info(
            f"TARGET (map): [{target_x:.2f}, {target_y:.2f}] | "
            f"POSE (map): [{robot_map_x:.2f}, {robot_map_y:.2f}] | "
            f"DIST: {dist:.2f}m | ANGLE_ERR: {math.degrees(angle_diff):.1f}deg",
            throttle_duration_sec=0.5
        )

        if dist < 0.3: #Condition te say that we reached the point
            self.get_logger().info(f"REACHED WAYPOINT: [{target_x:.2f}, {target_y:.2f}]")
            self.current_path.pop(0)
            return

        msg = Twist()
        if abs(angle_diff) > 0.2: 
            msg.angular.z = 0.4 if angle_diff > 0 else -0.4
            msg.linear.x = 0.0
        else: 
            msg.linear.x = 0.15 
            msg.angular.z = angle_diff * 1.0 

        self.cmd_vel_pub.publish(msg)

    def map_callback(self, msg):
        """ Stores map data and triggers simulation once. """
        if self.map_data is None and msg.data:
            if not msg.data:
                return
            self.get_logger().info("Map received! Running RRT planning in 2 seconds...")
            self.map_data = msg
            self.destroy_subscription(self.map_sub)
            self.planning_timer=self.create_timer(5.0, self.run_planning_timer_callback)
            self.get_logger().info("Running RRT planning now...")

    def odom_callback(self, msg):
        """ Update the robot's current position and orientation. """
        # Position
        self.robot_pose['x'] = msg.pose.pose.position.x
        self.robot_pose['y'] = msg.pose.pose.position.y
        
        # Orientation (Quaternion to Euler Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_pose['yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def run_planning_timer_callback(self):
        if self.planning_timer:
            self.planning_timer.cancel()
            self.destroy_timer(self.planning_timer)
            self.planning_timer = None

        self.get_logger().info("Running RRT* calculation...")
        
        result = self.run_simulation()

        if result is None:
            self.get_logger().error("RRT* failed to initialize.")
            return

        path, planner = result 
    
        if path:
            self.current_path = path
            self.latest_path = path

            if planner and hasattr(planner, 'node_list'):
                all_nodes = planner.node_list 
                self.publish_tree(all_nodes)
            
            self.get_logger().info(f"Path found with {len(path)} points!")
            
            if self.control_timer is None:
                self.control_timer = self.create_timer(0.1, self.follow_path)
        else:
            self.get_logger().error("RRT failed to find a path. No tree to display.")
    
    
    def publish_tree(self, nodes):
        """
        Publishes the entire exploration tree as a series of lines in RViz.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rrt_tree"
        marker.id = 1
        marker.type = Marker.LINE_LIST 
        marker.action = Marker.ADD
    
        # Visual style: Thin blue lines
        marker.scale.x = 0.02 # Line width
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.6 
    
        for node in nodes:
            if len(node.parent_x) > 1:
                # Point 1: The current node
                p1 = Point()
                p1.x, p1.y, p1.z = float(node.x), float(node.y), 0.05
            
                # Point 2: Its parent 
                p2 = Point()
                p2.x, p2.y, p2.z = float(node.parent_x[-2]), float(node.parent_y[-2]), 0.05
            
                marker.points.append(p1)
                marker.points.append(p2)
            
        self.tree_pub.publish(marker)

    def run_simulation(self):
        """ Main logic to call the RRT algorithm. """
        rand_area = [0.1, 14.8, 0.1, 9.9] 
        start = [0.5, 5.0]  
        goal = [2.5, 5.0] #Goal to change

        self.latest_goal = goal
        self.get_logger().info(f"Planning from {start} to {goal}...")
        start_time = time.time()

        # RRT
        planner = RRT(
            start=start,
            goal=goal,
            rand_area=rand_area,
            step_size=self.step_size,
            map_data=self.map_data
        )
        
        path = planner.planning() 
        end_time = time.time()

        if path:
            self.get_logger().info(f"Path found in {end_time - start_time:.4f}s")
            self.latest_path = path  
            self.current_path = path 
            self.get_logger().info(f"Path loaded: {len(self.current_path)} points.")
            return path, planner 
        else:
            self.get_logger().warn("RRT failed to find a valid path.")
            return None, planner


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
        marker.pose.position.z = 0.5 
        
        # Size of the sphere (0.3m)
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # Color: Bright Red
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0 
        
        self.marker_pub.publish(marker)
    
    def stop_robot(self):
        """ Publishes a zero velocity command to safely stop the robot. """
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

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