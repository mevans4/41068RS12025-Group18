import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from PyQt5 import QtWidgets
from drone_ui.drone_ui_code import Ui_MainWindow
from std_msgs.msg import Bool

class DroneUINode(Node):
    def __init__(self):
        super().__init__('drone_ui_node')

        # Publishers
        self.start_pub = self.create_publisher(Bool, '/drone/cmd/start', 10)
        self.stop_pub = self.create_publisher(Bool, '/drone/cmd/stop', 10)
        self.home_pub = self.create_publisher(Bool, '/drone/cmd/return_home', 10)
        self.height_pub = self.create_publisher(Float32, '/drone/cmd/height', 10)
        self.tree_pub = self.create_publisher(Float32, '/drone/cmd/tree_select', 10)

        # Subscribers
        self.create_subscription(String, '/drone/status', self.status_callback, 10)
        self.create_subscription(Float32, '/drone/tree_width', self.width_callback, 10)

        # Setup Qt
        self.app = QtWidgets.QApplication(sys.argv)
        self.window = QtWidgets.QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.window)

        # Connect UI buttons to ROS functions
        self.ui.Start.clicked.connect(self.start_drone)
        self.ui.Stop.clicked.connect(self.stop_drone)
        self.ui.ReturnHome.clicked.connect(self.return_home)
        self.ui.HeightControl.valueChanged.connect(self.set_height)
        self.ui.pushButton.clicked.connect(self.get_width)

        self.window.show()

    # --- Publisher callbacks ---
    def start_drone(self):
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)
        self.get_logger().info("Start command sent")

    def stop_drone(self):
        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)
        self.get_logger().info("Stop command sent")

    def return_home(self):
        msg = Bool()
        msg.data = True
        self.home_pub.publish(msg)
        self.get_logger().info("Return Home command sent")

    def set_height(self, value):
        msg = Float32()
        msg.data = float(value)
        self.height_pub.publish(msg)
        self.get_logger().info(f"Height set to {value:.2f} m")

    def get_width(self):
        msg = Float32()
        msg.data = float(self.ui.TreeSelect.value())
        self.tree_pub.publish(msg)
        self.get_logger().info(f"Tree {msg.data} selected")

    # --- Subscriber callbacks ---
    def status_callback(self, msg):
        self.ui.statusbar.showMessage(msg.data)

    def width_callback(self, msg):
        self.ui.label.setText(f"Width: {msg.data:.2f} m")

    def run(self):
        sys.exit(self.app.exec_())

def main():
    rclpy.init()
    node = DroneUINode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
