import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from . import ESP32, Turtlebot

class MoveToTag(Node):
    def __init__(self):
        super().__init__('move_to_tag')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            ESP32().ARUCO_TAG,
            self.aruco_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, Turtlebot().CMD_VEL, 10)
        self.get_logger().info("MoveToTag node has been started.")
        self.linear_speed = 0.2
        self.angular_speed = 0.5

    def aruco_callback(self, msg):
        if len(msg.data) < 3:
            self.get_logger().warn("Invalid ArUco tag data received.")
            return

        x, y, z = msg.data  # Assuming x, y, z are the tag's position in meters
        twist = Twist()

        # If the tag is far, move forward
        if z > 1.0:
            twist.linear.x = self.linear_speed
            twist.angular.z = -self.angular_speed * x  # Turn towards the tag
        # If the tag is close, stop
        elif z < 0.3:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        # If the tag is at an intermediate distance, curve towards it
        else:
            twist.linear.x = self.linear_speed * 0.5
            twist.angular.z = -self.angular_speed * x

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToTag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()