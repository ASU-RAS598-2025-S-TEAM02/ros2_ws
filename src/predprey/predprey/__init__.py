import rclpy.qos

QOS_PROFILE = rclpy.qos.QoSProfile(
    history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
)

class TopicBuilder:
    def __init__(self, base):
        self.base = base

    def __truediv__(self, other):
        return TopicBuilder(f"{self.base}/{other.strip('/')}")

    def __str__(self):
        return self.base


class Hardware:
    def __init__(self, name, num="05"):
        self.name = name
        self.num = num

    def __str__(self):
        return f"/{self.name}_{self.num}"

    def __truediv__(self, other):
        return TopicBuilder(self.__str__()) / other


class ESP32(Hardware):
    def __init__(self):
        super().__init__("esp")

        #############
        # NAMESPACE #
        #############
        self.NAMESPACE = str(self)

        # PUBLISHER TOPICS
        self.IMU = self / "imu_data"
        self.RELATIVE_POSITION = self / "relative_position"


class Turtlebot(Hardware):
    def __init__(self):
        super().__init__("turtlebot")

        #############
        # NAMESPACE #
        #############
        self.NAMESPACE = str(self)

        ####################
        # PUBLISHER TOPICS #
        ####################

        self.IMU = self / "imu"

        # OAKD CAMERA
        self._oakd = self / "oakd" / "rgb" / "preview"
        self.CAMERA = self._oakd / "image_raw"
        self.CAMERA_INFO = self._oakd / "camera_info"

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
    print()

    print("Turtlebot Topics:")
    print(f"  Namespace: {turtlebot.NAMESPACE}")
    print(f"  IMU: {turtlebot.IMU}")
    print(f"  Camera Image: {turtlebot.CAMERA}")
    print(f"  Camera Info: {turtlebot.CAMERA_INFO}")
    print(f"  Distance to ArUco: {turtlebot.DIST_TO_ARUCO}")
    print(f"  LIDAR: {turtlebot.LIDAR}")
    print(f"  SLAM: {turtlebot.SLAM}")
    print(f"  Pose: {turtlebot.POSE}")
    print(f"  Command Velocity: {turtlebot.CMD_VEL}")