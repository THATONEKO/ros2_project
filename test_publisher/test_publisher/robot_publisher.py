
#!/usr/bin/env python3
"""
Script to autonomously navigate Robot
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math


class AutonomousRobot(Node):
    def __init__(self):
        super().__init__("autonomous_robot")
        self.publisher = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )
        self.subscriber = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/robot_base_controller/odom',
            self.odom_callback,
            10
        )

        self.subscription  # prevent unused variable warning

        self.cmd = Twist()
        self.safe_distance = 0.9  # Distance to avoid obstacles in meters
        self.obstacle_on_left = False
        self.obstacle_on_right = False
        self.adjust = 0 

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians
    
    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self.get_logger().info(f"Orientation (Euler angles): Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")    

    def lidar_callback(self, msg):
        # Check for obstacles in front of the robot
        front_distances = msg.ranges[len(msg.ranges) // 3:2 * len(msg.ranges) // 3]

        half_index = len(front_distances) // 2
        left_scan = front_distances[:half_index]
        right_scan = front_distances[half_index:]

        self.obstacle_on_left = min(left_scan) < self.safe_distance
        self.obstacle_on_right = min(right_scan) < self.safe_distance

        self.get_logger().info(f"right scan: {left_scan}")
        self.get_logger().info(f"left scan: {right_scan}")

    def run(self):
        while rclpy.ok():
            if self.obstacle_on_left or self.obstacle_on_right:
                self.get_logger().info(f"obstacle there")
                self.avoid_obstacle() 
                if not self.obstacle_on_right or self.obstacle_on_left:
                    self.adjust = 0
            else:
                self.get_logger().info(f"no obstacle")
                self.move_forward()
            rclpy.spin_once(self)

    def move_forward(self):
        self.adjust = 0
        self.cmd.linear.x = 0.5
        self.cmd.angular.z = 0.0
        self.publisher.publish(self.cmd)

    def avoid_obstacle(self):
        self.cmd.linear.x = 0.0
        if self.obstacle_on_left:
            self.cmd.angular.z = 0.5  # Turn right if there's an obstacle on the left
            self.get_logger().info(f"turning left: {self.cmd.angular.z}")
            self.adjust += self.cmd.angular.z
            self.get_logger().info(f"total rotation: {self.adjust}")
            
            
        elif self.obstacle_on_right:
            self.cmd.angular.z = -0.5  # Turn left if there's an obstacle on the right
            self.get_logger().info(f"turning right: {self.cmd.angular.z}")
            self.adjust -= self.cmd.angular.z
            self.get_logger().info(f"total rotation: {self.adjust}")
           
            
            
        self.publisher.publish(self.cmd)

    def back_to_path(self):
        self.cmd.angular.x = 0.0
        self.cmd.angular.z = -(self.adjust)
        self.adjust = 0
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