import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException

class RobotLocation(Node):
    def __init__(self):
        super().__init__("robot_location")

        # Declare goal latitude and longitude parameters
        # self.declare_parameter("goal_latitude", rclpy.Parameter.Type.DOUBLE)
        # self.declare_parameter("goal_longitude", rclpy.Parameter.Type.DOUBLE)

        # Get goal location from world coordinates yaml file
        # goal_lat = self.get_parameter("goal_latitude")
        # goal_long = self.get_parameter("goal_longitude")

        # Print goal location
        # self.get_logger().info(
        #     "goal location: %f %f"
        #     % (
        #         goal_lat.value,
        #         goal_long.value,
        #     )
        # )

        # Get robot's current location using tf2_ros
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create a timer to get the robot's location periodically
        timer_period = 1.0  # Adjust this value to change the time interval (in seconds)
        self.timer = self.create_timer(timer_period, self.get_robot_location)

    def get_robot_location(self):
        try:
            transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            self.get_logger().info(f"Robot's location: ({robot_x}, {robot_y})")
        except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Failed to get robot's location: {e}")

def main(args=None):
    rclpy.init(args=args)
    robot_location = RobotLocation()
    rclpy.spin(robot_location)
    robot_location.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()