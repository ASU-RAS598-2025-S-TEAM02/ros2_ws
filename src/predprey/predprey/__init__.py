from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

QOS_PROFILE = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT
)

class Hardware:
    ID = "05"

class ESP32:
    NAMESPACE = f"/esp_{Hardware.ID}/"
    
    # SUBSCRIPTION TOPICS
    IMU = NAMESPACE + "imu_data"

    # PUBLISHER TOPICS
    RELATIVE_POSITION = NAMESPACE + "rel_pos"

class Turtlebot:
    NAMESPACE = f"/rpi_{Hardware.ID}/"
    
    # SUBSCRIPTION TOPICS
    IMU = NAMESPACE + "imu"
    CAMERA = NAMESPACE + "oakd/rgb/preview/image_raw"
    LIDAR = NAMESPACE + "scan"
    SLAM = NAMESPACE + "map"
    POSE = NAMESPACE + "pose"
    
    # PUBLISHER TOPICS
    # TBD