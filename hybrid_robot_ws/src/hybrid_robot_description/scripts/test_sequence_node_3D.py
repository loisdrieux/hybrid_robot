#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import time

class Test3D(Node):
    def __init__(self):
        super().__init__('test_sequence_node')
        self.get_logger().info("Init test node")
        
        self.lift_pub = self.create_publisher(Float64MultiArray, '/lift_controller/commands', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_drone', 10)
        
        # Timer to let Gazebo launch
        self.create_timer(10.0, self.run_test)

    def run_test(self):
        if hasattr(self, 'done') and self.done:
            return
        self.done = True

        self.get_logger().info("Starting Sequence")

        # Lift
        lift_msg = Float64MultiArray()
        lift_msg.data = [2.0]
        self.get_logger().info("Lift")
        for _ in range(5):
            self.lift_pub.publish(lift_msg)
            time.sleep(0.1)
        
        time.sleep(5.0) # Wait that it has gone up

        # Moving forward
        move_msg = Twist()
        move_msg.linear.x = 0.3
        self.get_logger().info("Moving forward")
        for _ in range(50):
            self.cmd_pub.publish(move_msg)
            time.sleep(0.1)
        #Stopping
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Sequence Done")

def main(args=None):
    rclpy.init(args=args)
    node = Test3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()