#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSLogger(Node):
    def __init__(self):
        super().__init__('gps_logger')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

    def gps_callback(self, msg):
        self.get_logger().info(
            f"Latitude: {msg.latitude}, Longitude: {msg.longitude}, Altitude: {msg.altitude}"
        )

def main(args=None):
    rclpy.init(args=args)
    gps_logger = GPSLogger()
    rclpy.spin(gps_logger)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
