import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from predprey import Turtlebot
import time

class MoveToTag(Node):
    def __init__(self):
        super().__init__('move_to_tag')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            Turtlebot().DIST_TO_ARUCO,
            self.aruco_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, Turtlebot().CMD_VEL, 10)
        self.get_logger().info("MoveToTag node has been started.")
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.last_tag_time = time.time()

    def aruco_callback(self, msg):
        self.last_tag_time = time.time()  # Update the last tag detection time

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

    def spin_if_no_tag(self):
        current_time = time.time()
        if current_time - self.last_tag_time > 2.0:  # No tag detected for 2 seconds
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed * 0.2  # Slow spin
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToTag()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.spin_if_no_tag()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()