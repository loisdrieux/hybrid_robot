#!/usr/bin/env python3
import rclpy
import rrt 
import numpy as np
import time
import math

from rclpy.node import Node
from std_msgs.msg import Empty, Float64MultiArray 
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker 
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from rrt import RRT, collision
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.duration import Duration

class RRTPlannerNode(Node):
    def __init__(self):
        super().__init__('rrt_planner_node')
        
        self.robot_started = False
        self.planning_timer = None 
        self.start_time = None # Criteria for time of simulation
        self.total_air_points = 0 # Criteria for air points
        self.mission_completed = False 
        self.get_logger().info("System ready. Waiting for map, then use '/start_robot' to move.")
        
        # QoS Settings
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        tree_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        marker_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Parameters
        self.step_size = 1.5  # CHANGE STEP SIZE HERE
        self.control_timer = None 
        self.map_data = None
        self.current_path = [] 
        self.latest_path = None 
        self.latest_goal = None
        # 2D Version: self.robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0} # 3D Version

        # Movement Constraints
        self.target_reached_dist = 0.25
        self.linear_speed = 0.15
        self.angular_speed = 0.4

        # Timer 
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.marker_timer = self.create_timer(1.0, self.timer_callback)

        # Subscribers
        self.start_sub = self.create_subscription(Empty, '/start_robot', self.start_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos) 
        self.odom_sub = self.create_subscription(Odometry, 'odom_drone', self.odom_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/rrt_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_drone', 10)
        self.marker_pub = self.create_publisher(Marker, 'goal_marker', marker_qos)
        self.tree_pub = self.create_publisher(Marker, '/rrt_tree', tree_qos)
        self.lift_pub = self.create_publisher(Float64MultiArray, '/lift_controller/commands', 10)
        
        self.get_logger().info("RRT Node started. Persistent publishing enabled.")

    def start_callback(self, msg):
        self.get_logger().info("Starting signal received, the robot will start moving")

        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info("Movement timer started!")

        self.robot_started = True

    def timer_callback(self):
        if self.latest_path is not None:
            self.publish_path(self.latest_path)
        if self.latest_goal is not None:
            self.publish_goal_marker(self.latest_goal)

        if not self.current_path and not self.mission_completed and self.start_time is not None:
            end_time = self.get_clock().now()
            duration = (end_time - self.start_time).nanoseconds / 1e9 # Need to convert to seconds
            self.mission_completed = True
            self.get_logger().info(f"Movement Time: {duration:.2f} seconds")
            self.get_logger().info(f"Air Waypoints: {self.total_air_points}")

    def joint_callback(self, msg):
        """ English comment: Updates robot altitude by reading the prismatic joint state """
        try:
            if 'lift_joint' in msg.name:
                idx = msg.name.index('lift_joint')
                self.robot_pose['z'] = msg.position[idx]
        except (ValueError, IndexError):
            pass

    def follow_path(self):
        if not self.robot_started or not self.current_path:
            self.stop_robot()
            return

        # 2D Version: target_x, target_y = self.current_path[0]
        target_x, target_y, target_z = self.current_path[0] # 3D Version
    
        robot_map_x = self.robot_pose['x']
        robot_map_y = self.robot_pose['y']
        robot_map_z = self.robot_pose['z']

        # Distance calculation
        dx = target_x - robot_map_x
        dy = target_y - robot_map_y
        dist_xy = math.sqrt(dx**2 + dy**2)
        dist_z = abs(target_z - robot_map_z)
    
        # Angle calculation 
        angle_to_target = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(angle_to_target - self.robot_pose['yaw']), 
                        math.cos(angle_to_target - self.robot_pose['yaw']))

        # Debug point by point
        self.get_logger().info(
            f"TARGET: [{target_x:.2f}, {target_y:.2f}, {target_z:.2f}] | "
            f"POSE: [{robot_map_x:.2f}, {robot_map_y:.2f}, {robot_map_z:.2f}] | "
            f"DIST_XY: {dist_xy:.2f}m | DIST_Z: {dist_z:.2f}m",
            throttle_duration_sec=0.5
        )

        # Waypoint reached condition
        if dist_xy < 0.2 and dist_z < 0.2:
            self.get_logger().info(f"REACHED WAYPOINT: [{target_x:.2f}, {target_y:.2f}, {target_z:.2f}]")
            self.current_path.pop(0)
            return

        # Lift
        lift_msg = Float64MultiArray()
        lift_msg.data = [float(target_z)]
        self.lift_pub.publish(lift_msg)

        # Linear movement
        msg = Twist()
        if abs(angle_diff) > 0.2: 
            msg.angular.z = 0.4 if angle_diff > 0 else -0.4
            msg.linear.x = 0.0
        else: 
            msg.linear.x = 0.15 
            msg.angular.z = angle_diff * 1.0 

        self.cmd_vel_pub.publish(msg)

    def map_callback(self, msg):
        if self.map_data is None and msg.data:
            self.get_logger().info("Map received! Running RRT planning...")
            self.map_data = msg
            self.destroy_subscription(self.map_sub)
            self.planning_timer = self.create_timer(5.0, self.run_planning_timer_callback)

    def odom_callback(self, msg):
        self.robot_pose['x'] = msg.pose.pose.position.x
        self.robot_pose['y'] = msg.pose.pose.position.y
        self.robot_pose['z'] = msg.pose.pose.position.z 
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_pose['yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def run_planning_timer_callback(self):
        if self.planning_timer:
            self.planning_timer.cancel()
            self.destroy_timer(self.planning_timer)
            self.planning_timer = None

        self.get_logger().info("Running RRT 3D calculation...")
        result = self.run_simulation()

        if result is None:
            self.get_logger().error("RRT failed to initialize.")
            return

        path, planner = result 
    
        if path:
            self.current_path = path
            self.latest_path = path
            if planner and hasattr(planner, 'node_list'):
                self.publish_tree(planner.node_list)
            
            self.get_logger().info(f"Path found with {len(path)} points!")
            if self.control_timer is None:
                self.control_timer = self.create_timer(0.1, self.follow_path)
        else:
            self.get_logger().error("RRT failed to find a path.")
    
    def publish_tree(self, nodes):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rrt_tree"
        marker.id = 1
        marker.type = Marker.LINE_LIST 
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 0.5, 1.0, 0.6)
    
        for node in nodes:
            if len(node.parent_x) > 1:
                p1 = Point()
                # 2D Version: p1.z = 0.05
                p1.x, p1.y, p1.z = float(node.x), float(node.y), float(node.z) # 3D Version
            
                p2 = Point()
                p2.x = float(node.parent_x[-2])
                p2.y = float(node.parent_y[-2])
                p2.z = float(node.parent_z[-2])
            
                marker.points.append(p1)
                marker.points.append(p2)
            
        self.tree_pub.publish(marker)

    def run_simulation(self):
        rand_area = [0.1, 14.8, 0.1, 9.9, 0.0, 4.0]

        start = [0.5, 5.0, 0.0]  
        goal = [9.0, 9.0, 1.5]  # CHANGE GOAL HERE

        self.latest_goal = goal
        self.get_logger().info(f"Planning 3D from {start} to {goal}...")
        start_time = time.time()

        planner = RRT(
            start=start,
            goal=goal,
            rand_area=rand_area,
            step_size=self.step_size,
            map_data=self.map_data,
            z_penalty=15.0  # CHANGE PENALTY HERE 
        )
        
        path = planner.planning() 
        end_time = time.time()

        if path:
            self.get_logger().info(f"Path found in {end_time - start_time:.4f}s")
            self.latest_path = path  
            self.current_path = path 
            self.total_air_points = sum(1 for p in path if p[2] > 0.10) #Condition that says we are not on the ground anymore
            self.get_logger().info(f"There are {self.total_air_points} points in the air out of {len(path)} total points.")
            return path, planner 
        else:
            self.get_logger().warn("RRT failed to find a valid path.")
            return None, planner

    def publish_path(self, rrt_path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for point in rrt_path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            # 2D Version: pose.pose.position.z = 0.1
            pose.pose.position.z = float(point[2])
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

    def publish_goal_marker(self, goal):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(goal[0])
        marker.pose.position.y = float(goal[1])
        # 2D Version: marker.pose.position.z = 0.5
        marker.pose.position.z = float(goal[2]) 
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.0, 0.0, 1.0) 
        marker.lifetime = Duration(seconds=2).to_msg()
        self.marker_pub.publish(marker)
    
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = msg.angular.z = 0.0
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