#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import carla # Optional, only if you want to pick a spawn point

class StaticGoalPublisher(Node):
    def __init__(self):
        super().__init__('static_goal_publisher')
        self.pub = self.create_publisher(PoseStamped, '/carla/agent_0/goal', 10)
        # Use a timer to publish once after a delay (e.g., 5s)
        self.timer = self.create_timer(5.0, self.publish_goal)
        self.goal_published = False
        self.get_logger().info("StaticGoalPublisher started. Will publish goal soon.")

    def publish_goal(self):
        if self.goal_published:
            self.timer.cancel() # Stop timer after publishing once
            return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        # --- DEFINE YOUR STATIC GOAL HERE ---
        # Example: A point further down the initial road in Town04
        goal.pose.position.x = 200.0
        goal.pose.position.y = 170.0 # Keep y the same for straight path initially
        goal.pose.position.z = 0.0
        
        goal.pose.orientation.w = 1.0 # Default orientation (facing positive X in goal frame)
        # ------------------------------------

        self.pub.publish(goal)
        self.goal_published = True
        self.get_logger().info(f"Published static goal at ({goal.pose.position.x:.1f}, {goal.pose.position.y:.1f})")
        self.get_logger().info("Static goal published once. Timer stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = StaticGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
