import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import cv2.aruco as aruco

class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimator')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize ArUco dictionary and detection parameters
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        
        if hasattr(aruco, "DetectorParameters_create"):
            self.parameters = aruco.DetectorParameters_create()
        else:
            self.parameters = aruco.DetectorParameters()

        # Marker size in meters
        self.marker_length = 0.04  # 4 cm

        # Intrinsics will be loaded from CameraInfo topic
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False  # New flag to prevent crash

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10)

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/oakd/rgb/preview/camera_info',
            self.camera_info_callback,
            10)

        self.get_logger().info("ArUco Pose Estimator node has started (waiting for camera intrinsics)")

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info_received:
            return  # Ignore if already received

        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

        self.camera_info_received = True  # Set the flag
        self.get_logger().info(f" Camera intrinsics received: fx={self.camera_matrix[0,0]}, fy={self.camera_matrix[1,1]}")

    def image_callback(self, msg: Image):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn(" Waiting for camera intrinsics before processing images...")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(cv_image, corners, ids)

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                cv2.drawFrameAxes(
                    cv_image, self.camera_matrix, self.dist_coeffs,
                    rvecs[i], tvecs[i], self.marker_length * 0.5)

                tvec = tvecs[i][0]
                x, y, z = tvec[0], tvec[1], tvec[2]

                # Calculate depth
                depth_meters = z

                self.get_logger().info(
                    f" Marker ID: {ids[i][0]}\n"
                    f"  Position (tvec): x={x:.3f}m, y={y:.3f}m, z={z:.3f}m\n"
                    f"  Depth (from camera): {depth_meters:.3f} meters",
                    throttle_duration_sec=1.0)

        cv2.imshow("ArUco Pose Estimation", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    node = ArucoPoseEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()