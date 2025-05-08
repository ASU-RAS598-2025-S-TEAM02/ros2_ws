import rclpy.qos

QOS_PROFILE = rclpy.qos.QoSProfile(
    history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
)

class TopicPath:
    def __init__(self, base):
        self.path = base

    def __truediv__(self, other):
        return TopicPath(f"{self.path.rstrip('/')}/{other.strip('/')}")

    def __str__(self):
        return self.path

    def __repr__(self):
        return self.path

class Hardware:
    def __init__(self, name):
        self._topic = TopicPath(f"/{name}")
        self.NAMESPACE = str(self._topic)

    def __truediv__(self, other):
        return self._topic / other

    def __repr__(self):
        topics = {k: v for k, v in self.__dict__.items() if not k.startswith('_') and k != 'NAMESPACE'}
        return f"{self.__class__.__name__}(NAMESPACE='{self.NAMESPACE}', Topics={list(topics.values())})"

class ESP32(Hardware):
    def __init__(self):
        super().__init__("esp_05")

        #############
        # NAMESPACE #
        #############
        # Access via self.NAMESPACE or ESP32.NAMESPACE

        ####################
        # PUBLISHER TOPICS #
        ####################
        self.IMU = self / "imu_data"
        self.RELATIVE_POSITION = self / "relative_pos"

class Turtlebot(Hardware):
    def __init__(self):
        super().__init__("turtlebot_05")

        #############
        # NAMESPACE #
        #############
        # Access via self.NAMESPACE or Turtlebot.NAMESPACE

        ####################
        # PUBLISHER TOPICS #
        ####################
        self.IMU = self / "imu"

        oakd = self / "oakd" / "rgb" / "preview"
        self.CAMERA = oakd / "image_raw"
        self.CAMERA_INFO = oakd / "camera_info"

        self.DIST_TO_ARUCO = self / "marker_pos"
        self.LIDAR = self / "scan"
        self.SLAM = self / "map"
        self.POSE = self / "pose"

        #######################
        # SUBSCRIPTION TOPICS #
        #######################
        self.CMD_VEL = self / "cmd_vel"

# Topic Checks
if __name__ == "__main__":
    esp = ESP32()
    turtlebot = Turtlebot()

    print("ESP32 Topics:")
    print(f"  Namespace: {esp.NAMESPACE}")
    print(f"  IMU: {esp.IMU}")
    print(f"  Relative Position: {esp.RELATIVE_POSITION}\n")

    print("Turtlebot Topics:")
    print(f"  Namespace: {turtlebot.NAMESPACE}")
    print(f"  IMU: {turtlebot.IMU}")
    print(f"  Camera Image: {turtlebot.CAMERA}")
    print(f"  Camera Info: {turtlebot.CAMERA_INFO}")
    print(f"  Distance to ArUco: {turtlebot.DIST_TO_ARUCO}")
    print(f"  LIDAR: {turtlebot.LIDAR}")
    print(f"  SLAM: {turtlebot.SLAM}")
    print(f"  Pose: {turtlebot.POSE}")
    print(f"  Command Velocity: {turtlebot.CMD_VEL}\n")

    print("Hardware Overview:")
    print(turtlebot)
    print(esp)