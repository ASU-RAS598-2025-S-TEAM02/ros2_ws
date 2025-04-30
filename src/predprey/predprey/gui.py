import sys
import numpy as np
import PyQt5.QtWidgets as qw
import PyQt5.QtCore as qc
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
from PyQt5.QtGui import QImage, QPixmap, QPainter, QBrush, QColor
import time
from . import ESP32, Turtlebot, QOS_PROFILE

# GUI FOR: IMU, CAMERA, SLAM

class CameraWindow(qw.QWidget):
    def __init__(self, bridge, topic, node, parent=None):
        super(CameraWindow, self).__init__(parent)
        self.setWindowTitle("Camera Image Viewer")
        self.setGeometry(100, 100, 640, 480)
        self.layout = qw.QVBoxLayout()
        self.image_label = qw.QLabel(self)
        self.layout.addWidget(self.image_label)
        self.setLayout(self.layout)
        self.bridge = bridge
        self.node = node
        self.active = True
        self.last_update = time.time()

        self.image_subscription = self.node.create_subscription(
            Image,
            topic,
            self.camera_listener_callback,
            QOS_PROFILE
        )

        # Status timer
        self.status_timer = qc.QTimer()
        self.status_timer.timeout.connect(self.check_status)
        self.status_timer.start(1000)

    def check_status(self):
        if time.time() - self.last_update > 2.0:  # 2 seconds timeout
            self.active = False
            self.image_label.setText("Camera feed not available")
        else:
            self.active = True

    def camera_listener_callback(self, msg):
        try:
            self.last_update = time.time()
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, channels = cv_image.shape
            bytes_per_line = channels * width
            q_img = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
            self.image_label.setPixmap(QPixmap.fromImage(q_img))
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert camera image: {e}")
            self.active = False

class MapWindow(qw.QWidget):
    def __init__(self, node, parent=None):
        super(MapWindow, self).__init__(parent)
        self.setWindowTitle("Occupancy Grid Map Viewer")
        self.setGeometry(200, 200, 640, 480)
        self.image_label = qw.QLabel(self)
        self.status_label = qw.QLabel("Map: Waiting for data...", self)
        layout = qw.QVBoxLayout()
        layout.addWidget(self.status_label)
        layout.addWidget(self.image_label)
        self.setLayout(layout)
        self.node = node
        self.active = False
        self.last_update = 0

        self.subscription = self.node.create_subscription(
            OccupancyGrid,
            Turtlebot().SLAM,
            self.map_callback,
            QOS_PROFILE
        )

        self.robot_position = (0, 0)
        self.resolution = 0.05
        self.map_width = 640
        self.map_height = 480
        self.map_origin = (0.0, 0.0)

        # Status timer
        self.status_timer = qc.QTimer()
        self.status_timer.timeout.connect(self.check_status)
        self.status_timer.start(1000)

    def check_status(self):
        if time.time() - self.last_update > 5.0:  # 5 seconds timeout for map
            self.active = False
            self.status_label.setText("Map: Not receiving updates")
            self.status_label.setStyleSheet("color: red;")
        else:
            self.active = True
            self.status_label.setText("Map: Active")
            self.status_label.setStyleSheet("color: green;")

    def map_callback(self, msg):
        try:
            self.last_update = time.time()
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin = msg.info.origin.position
            self.map_origin = (origin.x, origin.y)
            self.resolution = resolution

            data = np.array(msg.data, dtype=np.int8).reshape((height, width))

            img = np.zeros((height, width), dtype=np.uint8)
            img[data == 0] = 255  # Free space
            img[data == 100] = 0   # Occupied
            img[data == -1] = 127  # Unknown

            img_resized = cv2.resize(img, (640, 480), interpolation=cv2.INTER_NEAREST)
            q_img = QImage(img_resized.data, img_resized.shape[1], img_resized.shape[0], 
                          img_resized.strides[0], QImage.Format_Grayscale8)
            pixmap = QPixmap.fromImage(q_img)

            # Transform robot world position to pixel coordinates
            robot_x, robot_y = self.robot_position
            map_x = int((robot_x - self.map_origin[0]) / resolution * (640 / width))
            map_y = int((height - (robot_y - self.map_origin[1]) / resolution) * (480 / height))

            painter = QPainter(pixmap)
            painter.setBrush(QBrush(QColor(255, 0, 0)))
            painter.drawEllipse(map_x - 5, map_y - 5, 10, 10)
            painter.end()

            self.image_label.setPixmap(pixmap)
            self.active = True
        except Exception as e:
            self.node.get_logger().error(f"Map processing error: {e}")
            self.active = False

class GraphView(qw.QWidget):
    def __init__(self, name='Name', title='Title', graph_title='Graph Title', parent=None, node=None, logger=None):
        super(GraphView, self).__init__(parent)
        self.name = name
        self.graph_title = graph_title
        self.fig = Figure((5.0, 3.0), dpi=100)
        self.axes = self.fig.add_subplot(111)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.layout = qw.QVBoxLayout()
        
        # Status display
        self.status_label = qw.QLabel("Status: RPI: Waiting... | ESP: Waiting... | Pose: Waiting...")
        self.layout.addWidget(self.status_label)
        self.layout.addWidget(self.toolbar)
        self.layout.addWidget(self.canvas)

        # Control buttons
        self.query_button = qw.QPushButton("Get Data at Time", self)
        self.query_button.clicked.connect(self.get_data_at_time)
        self.layout.addWidget(self.query_button)

        self.camera_button = qw.QPushButton("Subscribe to Camera", self)
        self.camera_button.clicked.connect(self.subscribe_to_camera)
        self.layout.addWidget(self.camera_button)

        self.map_button = qw.QPushButton("Subscribe to Map", self)
        self.map_button.clicked.connect(self.subscribe_to_map)
        self.layout.addWidget(self.map_button)

        self.setLayout(self.layout)
        self.canvas.show()

        self.node = node
        self.logger = logger
        self.history_rpi = np.zeros((0, 4))
        self.history_esp = np.zeros((0, 4))
        
        # Status tracking
        self.rpi_active = False
        self.esp_active = False
        self.pose_active = False
        self.last_rpi_time = 0
        self.last_esp_time = 0
        self.last_pose_time = 0
        self.timeout_threshold = 2.0

        # Initialize subscriptions
        self.initialize_subscriptions()

        self.bridge = CvBridge()
        self.camera_window = None
        self.map_window = None

        # Setup timers
        self.setup_timers()

    def initialize_subscriptions(self):
        """Initialize all ROS subscriptions with error handling"""
        try:
            self.rpi_subscription = self.node.create_subscription(
                Imu,
                Turtlebot().IMU,
                self.rpi_listener_callback,
                QOS_PROFILE
            )
        except Exception as e:
            self.logger.error(f"Failed to create RPI subscription: {e}")
            self.rpi_active = False

        try:
            self.esp_subscription = self.node.create_subscription(
                Imu,
                ESP32().IMU,
                self.esp_listener_callback,
                QOS_PROFILE
            )
        except Exception as e:
            self.logger.error(f"Failed to create ESP subscription: {e}")
            self.esp_active = False

        try:
            self.pose_subscription = self.node.create_subscription(
                PoseWithCovarianceStamped,
                Turtlebot().POSE,
                self.pose_callback,
                QOS_PROFILE
            )
        except Exception as e:
            self.logger.error(f"Failed to create pose subscription: {e}")
            self.pose_active = False

    def setup_timers(self):
        """Setup all QTimers with error handling"""
        try:
            self.graph_timer = qc.QTimer()
            self.graph_timer.setTimerType(qc.Qt.PreciseTimer)
            self.graph_timer.timeout.connect(self.update_graph)
            self.graph_timer.start(100)
        except Exception as e:
            self.logger.error(f"Failed to setup graph timer: {e}")

        try:
            self.status_timer = qc.QTimer()
            self.status_timer.timeout.connect(self.update_status)
            self.status_timer.start(1000)
        except Exception as e:
            self.logger.error(f"Failed to setup status timer: {e}")

    def update_status(self):
        """Update the status of all components"""
        current_time = time.time()
        
        # Check RPI status
        rpi_timed_out = (current_time - self.last_rpi_time) > self.timeout_threshold if self.last_rpi_time > 0 else True
        self.rpi_active = not rpi_timed_out
        
        # Check ESP status
        esp_timed_out = (current_time - self.last_esp_time) > self.timeout_threshold if self.last_esp_time > 0 else True
        self.esp_active = not esp_timed_out
        
        # Check pose status
        pose_timed_out = (current_time - self.last_pose_time) > self.timeout_threshold if self.last_pose_time > 0 else True
        self.pose_active = not pose_timed_out
        
        # Update status label
        status_text = (
            f"Status: RPI: {'Active' if self.rpi_active else 'Inactive'} | "
            f"ESP: {'Active' if self.esp_active else 'Inactive'} | "
            f"Pose: {'Active' if self.pose_active else 'Inactive'}"
        )
        self.status_label.setText(status_text)
        
        # Set color based on overall status
        if not self.rpi_active and not self.esp_active:
            self.status_label.setStyleSheet("color: red;")
        elif not self.rpi_active or not self.esp_active:
            self.status_label.setStyleSheet("color: orange;")
        else:
            self.status_label.setStyleSheet("color: green;")

    def subscribe_to_camera(self):
        try:
            if self.camera_window is None or not self.camera_window.isVisible():
                self.camera_window = CameraWindow(self.bridge, Turtlebot().CAMERA, self.node)
            self.camera_window.show()
        except Exception as e:
            self.logger.error(f"Failed to open camera window: {e}")
            qw.QMessageBox.warning(self, "Error", "Could not open camera window")

    def subscribe_to_map(self):
        try:
            if self.map_window is None or not self.map_window.isVisible():
                self.map_window = MapWindow(self.node)
            self.map_window.show()
        except Exception as e:
            self.logger.error(f"Failed to open map window: {e}")
            qw.QMessageBox.warning(self, "Error", "Could not open map window")

    def pose_callback(self, msg):
        try:
            self.last_pose_time = time.time()
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.logger.info(f"Pose: x={x}, y={y}", throttle_duration_sec=1)
            self.pose_active = True

            if self.map_window and self.map_window.isVisible():
                self.map_window.robot_position = (x, y)
        except Exception as e:
            self.logger.error(f"Pose callback error: {e}")
            self.pose_active = False

    def rpi_listener_callback(self, msg):
        try:
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.last_rpi_time = current_time
            if not hasattr(self, 't0_rpi'):
                self.t0_rpi = current_time
            t = current_time - self.t0_rpi
            self.history_rpi = np.vstack([self.history_rpi, [t, msg.linear_acceleration.x, 
                                                           msg.linear_acceleration.y, msg.linear_acceleration.z]])
            self.rpi_active = True
        except Exception as e:
            self.logger.error(f"RPI callback error: {e}")
            self.rpi_active = False

    def esp_listener_callback(self, msg):
        try:
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.last_esp_time = current_time
            if not hasattr(self, 't0_esp'):
                self.t0_esp = current_time
            t = current_time - self.t0_esp
            self.history_esp = np.vstack([self.history_esp, [t, msg.linear_acceleration.x, 
                                                           msg.linear_acceleration.y, msg.linear_acceleration.z]])
            self.esp_active = True
        except Exception as e:
            self.logger.error(f"ESP callback error: {e}")
            self.esp_active = False

    def update_graph(self):
        try:
            self.axes.clear()
            
            # Plot RPI data if available
            if self.rpi_active and len(self.history_rpi) > 0:
                self.axes.plot(self.history_rpi[:, 0], self.history_rpi[:, 1], 
                             label="RPI IMU" if self.esp_active else "RPI IMU (Only Active Sensor)")
            
            # Plot ESP data if available
            if self.esp_active and len(self.history_esp) > 0:
                self.axes.plot(self.history_esp[:, 0], self.history_esp[:, 1], 
                             label="ESP IMU" if self.rpi_active else "ESP IMU (Only Active Sensor)",
                             linestyle='--')
            
            # Handle case where no data is available
            if not self.rpi_active and not self.esp_active:
                self.axes.text(0.5, 0.5, 'No active IMU data', 
                              horizontalalignment='center',
                              verticalalignment='center',
                              transform=self.axes.transAxes)
                self.axes.set_xlim(0, 1)
                self.axes.set_ylim(0, 1)
            
            self.axes.set_title(self.graph_title)
            self.axes.set_xlabel('Time [s]')
            self.axes.set_ylabel('Acceleration [m/s²]')
            self.axes.grid(True)
            if self.rpi_active or self.esp_active:
                self.axes.legend()
            
            self.canvas.draw()
        except Exception as e:
            self.logger.error(f"Graph update error: {e}")
            # Attempt to recover by recreating the figure
            try:
                self.fig.clf()
                self.axes = self.fig.add_subplot(111)
                self.canvas.draw()
            except Exception as e2:
                self.logger.error(f"Failed to recover figure: {e2}")

        # Process ROS events
        try:
            rclpy.spin_once(self.node, timeout_sec=0)
        except Exception as e:
            self.logger.error(f"ROS spin error: {e}")

    def get_data_at_time(self):
        try:
            t, ok = qw.QInputDialog.getDouble(self, "Enter Time", "Enter time value t (in seconds):")
            if ok:
                rpi_val = esp_val = None
                
                if self.rpi_active and len(self.history_rpi) > 0:
                    idx_rpi = np.argmin(np.abs(self.history_rpi[:, 0] - t))
                    rpi_val = self.history_rpi[idx_rpi, 1]
                
                if self.esp_active and len(self.history_esp) > 0:
                    idx_esp = np.argmin(np.abs(self.history_esp[:, 0] - t))
                    esp_val = self.history_esp[idx_esp, 1]
                
                message = f"At time {t}s:\n"
                if rpi_val is not None:
                    message += f"RPI x: {rpi_val}\n"
                else:
                    message += "RPI data not available\n"
                
                if esp_val is not None:
                    message += f"ESP x: {esp_val}"
                else:
                    message += "ESP data not available"
                
                qw.QMessageBox.information(self, "Data", message)
        except Exception as e:
            self.logger.error(f"Data query error: {e}")
            qw.QMessageBox.warning(self, "Error", "Could not retrieve data at specified time")

class IMUDataLogger(Node):
    def __init__(self):
        super().__init__('imu_data_logger')

class Widget(qw.QWidget):
    def __init__(self, *args, subscriber_node, **kwargs):
        super(Widget, self).__init__(*args, **kwargs)
        self.graph = GraphView(node=subscriber_node, logger=subscriber_node.get_logger())
        layout = qw.QVBoxLayout()
        layout.addWidget(qw.QLabel('IMU Data Logger'))
        layout.addWidget(self.graph)
        self.setLayout(layout)

class MainWindow(qw.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setWindowTitle('IMU & Map Viewer')

def main(args=None):
    app = qw.QApplication(sys.argv)
    rclpy.init(args=args)
    
    try:
        imu_data_logger = IMUDataLogger()
        widget = Widget(subscriber_node=imu_data_logger)
        main_window = MainWindow()
        main_window.setCentralWidget(widget)
        main_window.show()

        try:
            app.exec_()
        except KeyboardInterrupt:
            pass
        finally:
            imu_data_logger.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"Fatal error during initialization: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
