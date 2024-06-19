#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from parc_robot_bringup.gps2cartesian import gps_to_cartesian


class GoalLocation(Node):
    def __init__(self):
        super().__init__("goal_location")

        # Declare goal latitude and longitude parameters
        self.declare_parameter("goal_latitude", 0.0)
        self.declare_parameter("goal_longitude", 0.0)

        # Get goal location from parameters
        self.goal_lat = self.get_parameter("goal_latitude").get_parameter_value().double_value
        self.goal_long = self.get_parameter("goal_longitude").get_parameter_value().double_value

        # Print goal location
        self.get_logger().info(
            "Goal location: %f %f"
            % (
                self.goal_lat,
                self.goal_long,
            )
        )

        # Call the gps_callback to print x and y coordinates
        self.gps_callback()

    def gps_callback(self):
        # Get the cartesian coordinates from the GPS coordinates
        x, y = gps_to_cartesian(self.goal_lat, self.goal_long)

        # Print cartesian coordinates
        self.get_logger().info(
            "The translation from the origin (0, 0) to the GPS location provided: %.3f %.3f"
            % (x, y)
        )


def main(args=None):
    rclpy.init(args=args)

    goal_location = GoalLocation()
    rclpy.spin(goal_location)

    goal_location.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
