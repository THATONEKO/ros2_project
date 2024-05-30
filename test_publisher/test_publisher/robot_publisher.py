#!/usr/bin/env python3
"""
Script to autonomously navigate Robot
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AutonomousRobot(Node):
    def __init__(self):
        super().__init__("autonomous_robot")
        self.publisher = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )
        self.subscriber = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 10
        )
        self.cmd = Twist()
        self.safe_distance = 0.5  # Distance to avoid obstacles in meters
        self.obstacle_on_left = False
        self.obstacle_on_right = False

    def lidar_callback(self, msg):
        # Check for obstacles in front of the robot
        front_distances = msg.ranges[len(msg.ranges) // 3:2 * len(msg.ranges) // 3]

        half_index = len(front_distances) // 2
        left_scan = front_distances[:half_index]
        right_scan = front_distances[half_index:]

        self.obstacle_on_left = min(left_scan) < self.safe_distance
        self.obstacle_on_right = min(right_scan) < self.safe_distance

        self.get_logger().info(f"Left scan: {left_scan}")
        self.get_logger().info(f"Right scan: {right_scan}")

    def run(self):
        while rclpy.ok():
            if self.obstacle_on_left or self.obstacle_on_right:
                self.avoid_obstacle()
            else:
                self.move_forward()
            rclpy.spin_once(self)

    def move_forward(self):
        self.cmd.linear.x = 0.5
        self.cmd.angular.z = 0.0
        self.publisher.publish(self.cmd)

    def avoid_obstacle(self):
        self.cmd.linear.x = 0.0
        if self.obstacle_on_left:
            self.cmd.angular.z = 0.5  # Turn right if there's an obstacle on the left
            self.get_logger().info(f"turning left: {self.cmd.angular.z}")
        elif self.obstacle_on_right:
            self.cmd.angular.z = -0.5  # Turn left if there's an obstacle on the right
            self.get_logger().info(f"turning right: {self.cmd.angular.z}")
        self.publisher.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    autonomous_robot = AutonomousRobot()
    try:
        autonomous_robot.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
