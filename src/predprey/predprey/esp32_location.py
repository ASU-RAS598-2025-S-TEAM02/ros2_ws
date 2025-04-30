import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.signal import butter, lfilter
from predprey import ESP32, QOS_PROFILE

class IMULocator(Node):
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
        
        # Simplified but strong filtering parameters
        self.filter_order = 3  # Lower order but still effective
        self.cutoff_frequency = 0.5  # Very low cutoff for strong filtering (Hz)
        self.sampling_rate = 50.0  # Hz
        
        # Create Butterworth filter
        self.b, self.a = butter(self.filter_order, self.cutoff_frequency / (0.5 * self.sampling_rate), btype='low')
        
        # Buffer for acceleration smoothing and bias estimation
        self.accel_buffer_size = 15
        self.accel_buffer = []
        
        # Parameters for drift correction
        self.velocity_decay = 0.97  # Stronger decay factor to reduce drift
        self.zero_velocity_threshold = 0.03  # Threshold for zero-velocity detection
        
        # Create timer for periodic drift correction
        self.declare_parameter('update_frequency', 50.0)
        update_freq = self.get_parameter('update_frequency').value
        period = 1.0 / update_freq
        self.timer = self.create_timer(period, self.drift_correction_callback)
        
    def drift_correction_callback(self):
        """Periodically correct drift in velocity and position estimates"""
        if not self.initialized:
            return
            
        # Zero-velocity update - if movement is very small, assume we're stationary
        velocity_magnitude = np.linalg.norm(self.current_velocity)
        if velocity_magnitude < self.zero_velocity_threshold:
            self.current_velocity = np.zeros(3)
            
    def imu_callback(self, msg: Imu):
        if not self.initialized:
            self.initial_pose = np.array([0.0, 0.0, 0.0])
            self.prev_time = self.get_clock().now()
            self.initialized = True
            self.accel_buffer = []
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        # Extract linear acceleration
        raw_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
        # Update buffer and maintain buffer size
        self.accel_buffer.append(raw_acceleration)
        if len(self.accel_buffer) > self.accel_buffer_size:
            self.accel_buffer.pop(0)
            
        # Skip processing if buffer isn't full yet
        if len(self.accel_buffer) < self.accel_buffer_size:
            return
            
        # Simplified, but effective filtering approach:
        
        # 1. Apply low-pass filter to raw data
        accel_array = np.array(self.accel_buffer)
        filtered_accel = lfilter(self.b, self.a, accel_array, axis=0)[-1]
        
        # 2. Estimate and remove bias (simplified bias correction)
        accel_mean = np.mean(self.accel_buffer, axis=0)
        bias_corrected_accel = filtered_accel - 0.9 * accel_mean
        
        # 3. Threshold small accelerations (noise rejection)
        bias_corrected_accel[np.abs(bias_corrected_accel) < 0.015] = 0.0
        
        # Apply velocity decay to combat drift
        self.current_velocity *= self.velocity_decay
        
        # Integrate acceleration to get velocity
        velocity_increment = bias_corrected_accel * dt
        self.current_velocity += velocity_increment
        
        # Limit maximum velocity to prevent extreme drift
        max_velocity = 1.5  # m/s - lower than before for stricter control
        velocity_magnitude = np.linalg.norm(self.current_velocity)
        if velocity_magnitude > max_velocity:
            self.current_velocity = (self.current_velocity / velocity_magnitude) * max_velocity

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
    node = IMULocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()