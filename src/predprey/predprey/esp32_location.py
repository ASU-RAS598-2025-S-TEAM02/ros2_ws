import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.signal import butter, lfilter
from . import ESP32, QOS_PROFILE

class IMUFilterNode(Node):
    def __init__(self):
        super().__init__('esp32_rel_pos')

        self.esp32 = ESP32()

        self.subscription = self.create_subscription(
            Imu,
            self.esp32.IMU,
            self.imu_callback,
            qos_profile=QOS_PROFILE
        )
        self.publisher = self.create_publisher(PoseStamped, self.esp32.RELATIVE_POSITION, 10)
        self.initialized = False
        self.initial_pose = None
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.current_position = np.array([1.0, 0.0, 0.0])
        self.prev_time = None

        # Filter parameters
        self.filter_order = 2
        self.cutoff_frequency = 5.0  # Hz
        self.sampling_rate = 50.0  # Hz
        self.b, self.a = butter(self.filter_order, self.cutoff_frequency / (0.5 * self.sampling_rate), btype='low')

    def imu_callback(self, msg: Imu):
        if not self.initialized:
            self.initial_pose = np.array([0.0, 0.0, 0.0])
            self.prev_time = self.get_clock().now()
            self.initialized = True
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        # Extract linear acceleration
        raw_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        # Apply low-pass filter
        filtered_acceleration = lfilter(self.b, self.a, raw_acceleration)

        # Integrate acceleration to get velocity
        self.current_velocity += filtered_acceleration * dt

        # Integrate velocity to get position
        self.current_position += self.current_velocity * dt

        # Publish relative position
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = self.current_position[0]
        pose_msg.pose.position.y = self.current_position[1]
        pose_msg.pose.position.z = self.current_position[2]
        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()