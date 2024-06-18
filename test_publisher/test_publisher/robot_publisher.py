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
import tf2_ros
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import NavSatFix
from parc_robot_bringup.gps2cartesian import gps_to_cartesian


class AutonomousRobot(Node):
    def __init__(self):
        super().__init__("autonomous_robot")
        self.publisher = self.create_publisher(Twist, "/robot_base_controller/cmd_vel_unstamped", 10)
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.subscription = self.create_subscription(Odometry, '/robot_base_controller/odom', self.odom_callback, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 1
        )
    
        
        # Create a timer to get the robot's location periodically 
        self.timer = self.create_timer(0.5, self.get_robot_location)
        
        self.distance_to_obstacle = None
        self.angle_to_obstacle = None
        self.robot_x = None  
        self.robot_y =None

        self.robot_yaw = None
        self.obstacle_x = None
        self.obstacle_y = None
        self.initial_obstacle_stored = False

        self.x = 0.0
        self.y = 0.0

        self.obstacles_faced = []
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           

        self.cmd = Twist()
        self.safe_distance = 0.9  # Distance to avoid obstacles in meters
        self.obstacle_on_left = False
        self.obstacle_on_right = False
        self.adjust = 0 

    def quaternion_to_euler(self, q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x, y, z, w = q.x, q.y, q.z, q.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


    def gps_callback(self, gps):

        # Get the cartesian coordinates from the GPS coordinates
        x, y = gps_to_cartesian(gps.latitude, gps.longitude)

        self.x = x
        self.y = y

        # Print cartesian coordinates
        self.get_logger().info(
            "The translation from the origin (0, 0) to the GPS location provided: %.3f %.3f"
            % (self.x, self.y)
        )    

    def get_robot_location(self):
        try:

            transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            self.robot_x = self.x
            self.robot_y = self.y
            robot_orientation = transform.transform.rotation
            euler_angles = self.quaternion_to_euler(robot_orientation)
            self.robot_yaw = euler_angles[2]  # Yaw is the rotation around the Z-axis
            
            
            if self.distance_to_obstacle is not None and self.angle_to_obstacle is not None:
                self.obstacle_x = self.robot_x + self.distance_to_obstacle * math.cos(self.robot_yaw + self.angle_to_obstacle)
                self.obstacle_y = self.robot_y + self.distance_to_obstacle * math.sin(self.robot_yaw + self.angle_to_obstacle)
                self.obstacles_faced.append(self.obstacle_y)
                self.get_logger().info(f"Obstacle y: {self.obstacles_faced}") 
                
                if not self.initial_obstacle_stored:
                    self.initial_obstacle_x = self.obstacle_x
                    self.initial_obstacle_y = self.obstacle_y
                    self.initial_obstacle_stored = True

                self.get_logger().info(f"Robot's location: ({self.robot_x}, {self.robot_y})")
                self.get_logger().info(f"Obstacle's location: ({self.obstacle_x}, {self.obstacle_y})")
                self.get_logger().info(f"Initial Obstacle's location: ({self.initial_obstacle_x}, {self.initial_obstacle_y})")
            else:
                self.get_logger().info("Waiting for LIDAR data...")

        except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Failed to get robot's location: {e}")

    def get_robot_coordinates(self):
        if self.robot_x is not None and self.robot_y is not None:
            self.get_logger().info(f"Robot's coordinates: ({self.robot_x}, {self.robot_y})")
        else:
            self.get_logger().info("Waiting for robot's location...")        

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation)
        self.get_logger().info(f"Orientation (Euler angles): Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

    def lidar_callback(self, msg):
        # Initialize variables for the closest obstacle within the safe distance
        min_distance = float('inf')
        min_index = -1

        # Find the closest obstacle within the safe distance
        for i, distance in enumerate(msg.ranges):
            if distance < self.safe_distance and distance < min_distance:
                min_distance = distance
                min_index = i

        # If an obstacle within the safe distance is found, calculate its angle
        if min_index != -1:
            angle_increment = msg.angle_increment
            angle_to_obstacle = msg.angle_min + min_index * angle_increment
            self.distance_to_obstacle = min_distance
            self.angle_to_obstacle = angle_to_obstacle
        else:
            self.distance_to_obstacle = None
            self.angle_to_obstacle = None

        # Check for obstacles in front of the robot
        front_distances = msg.ranges[len(msg.ranges) // 3:2 * len(msg.ranges) // 3]
        half_index = len(front_distances) // 2
        left_scan = front_distances[:half_index]
        right_scan = front_distances[half_index:]

        self.obstacle_on_left = any(distance < self.safe_distance for distance in left_scan)
        self.obstacle_on_right = any(distance < self.safe_distance for distance in right_scan)

        self.get_logger().info(f"Distance to obstacle: {self.distance_to_obstacle}")
        self.get_logger().info(f"Angle to obstacle: {self.angle_to_obstacle}")
        self.get_logger().info(f"Obstacle on left: {self.obstacle_on_left}")
        self.get_logger().info(f"Obstacle on right: {self.obstacle_on_right}")


    def y_axis_comparison(self):
        # Compare the y-axis values of the robot and the obstacle
        if self.robot_y is not None and self.obstacle_y is not None:
            if abs(self.robot_y - self.obstacle_y) < 0.07:  # Allowing a small tolerance
                # self.cmd.linear.x = 0.0  # Stop the robot
                
                if self.adjust < 0.0: 
                    
                    # self.cmd.linear.x = 0.0
                    # self.cmd.angular.z = 3.0
                    # self.adjust += self.cmd.angular.z
                    self.get_logger().info(f"adjusting: {self.adjust}")
                    if self.adjust >= 0.0:
                        self.obstacles_faced = []


                elif self.adjust > 0.0:
                    
                    # self.cmd.angular.x = -4.0
                    # self.adjust += self.cmd.angular.z
                    self.get_logger().info(f"adjusting: {self.adjust}")
                    if self.adjust <= 0.0:
                        self.obstacles_faced = []


                elif self.adjust == 0.0:
                    self.get_logger().info("cannot adjust")
                    self.initial_obstacle_y = 0.0             
                         

                self.get_logger().info("Robot y-axis is equal to obstacle y-axis")
                self.back_to_path
                

                return True
        return False

    def run(self):
        while rclpy.ok():
            if self.obstacle_on_left or self.obstacle_on_right:
                self.avoid_obstacle()

            if self.adjust != 0.0:
                self.back_to_path()    
                # if not self.obstacle_on_right or self.obstacle_on_left:
                #     self.adjust = 0
            

            else:
                self.move_forward()
            rclpy.spin_once(self)

    def move_forward(self):
        self.cmd.linear.x = 0.5
        self.cmd.angular.z = 0.0

        if self.y_axis_comparison():
            self.get_logger().info("Y-axis alignment with obstacle detected. Taking evasive action.")
            self.cmd.linear.x = 0.0  # Stop the robot

        self.publisher.publish(self.cmd)

    def avoid_obstacle(self):
        self.cmd.linear.x = 0.0

        if self.obstacle_on_left:
            self.cmd.angular.z = 0.5  # Turn right if there's an obstacle on the left
            self.get_logger().info(f"Turning right: {self.cmd.angular.z}")
            self.adjust += self.cmd.angular.z
            self.get_logger().info(f"Total rotation: {self.adjust}")
        # if not self.obstacle_on_left and self.adjust > 0:
        #         while self.adjust > 0.0:
        #             self.cmd.angular.x = -2.0
        #             self.get_logger().info(f"nice: {self.adjust}")
            
        elif self.obstacle_on_right:
            self.cmd.angular.z = -0.5  # Turn left if there's an obstacle on the right
            self.get_logger().info(f"Turning left: {self.cmd.angular.z}")
            self.adjust += self.cmd.angular.z
            self.get_logger().info(f"Total rotation: {self.adjust}")
        # if not self.obstacle_on_right and self.adjust < 0:
        #         while self.adjust < 0.0:
        #             self.cmd.angular.x = 2.0      
        

        self.publisher.publish(self.cmd)

    def back_to_path(self):
        if not self.obstacle_on_left and self.adjust > 0:
            if self.adjust > 0.0:
                self.cmd.linear.x = 0.5
                self.cmd.angular.z = -1.0
                self.adjust += self.cmd.angular.z
                self.get_logger().info(f"nice: {self.adjust}")
                if 1.0 > self.adjust > -1.0:
                    self.cmd.angular.z = 0.0
                    self.publisher.publish(self.cmd)
                    

                self.publisher.publish(self.cmd)
        elif not self.obstacle_on_right and self.adjust < 0:
            if self.adjust < 0.0:
                self.cmd.linear.x = 0.5
                self.cmd.angular.z = 2.0
                self.adjust += self.cmd.angular.z
                self.get_logger().info(f"nice: {self.adjust}")
                if 1.0 > self.adjust > -1.0:
                    self.cmd.angular.z = 0.0
                    self.publisher.publish(self.cmd)

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