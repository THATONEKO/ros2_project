#!/usr/bin/env python3
## Install the geographiclib 2.0 module for this code to work.
## To install geographiclib 2.0, copy the line below to your terminal.
## pip install geographiclib
## Any of the PARC competition tasks must be running for this code to work.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from parc_robot_bringup.gps2cartesian import gps_to_cartesian


class GPSGoal(Node):
    def __init__(self):
        super().__init__("gps_goal")

        # Subscribe to the gps topic once
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 1
        )

    def gps_callback(self, gps):

        # Get the cartesian coordinates from the GPS coordinates
        x, y = gps_to_cartesian(gps.latitude, gps.longitude)

        # Print cartesian coordinates
        self.get_logger().info(
            "The translation from the origin (0, 0) to the gps location provided: %.3f %.3f"
            % (x, y)
        )


def main(args=None):
    rclpy.init(args=args)

    gps_goal = GPSGoal()
    rclpy.spin(gps_goal)

    gps_goal.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
