import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from lidar_tree_detector.msg import TreeDetectionArray
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
        self.create_subscription(TreeDetectionArray, 'detected_trees', self.trees_callback, 10)

        # store latest detector results as list of dicts: [{'width':int,'x':int,'y':int}, ...]
        self.latest_trees = []

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

        # prompt for row/column when Get Width pressed
        self.ui.pushButton.clicked.connect(self.on_get_width_clicked)

        # mapping: columns (along y) and rows (x positions)
        # user-facing row numbers: 1=left (-4), 2=center (0), 3=right (4)
        self.row_to_x = {1: -4, 2: 0, 3: 4}
        # columns 1..6 map to y positions bottom->top
        self.col_to_y = {1: -10, 2: -6, 3: -2, 4: 2, 5: 6, 6: 10}

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

    # def get_width(self):
    #     msg = Float32()
    #     msg.data = float(self.ui.TreeSelect.value())
    #     self.tree_pub.publish(msg)
    #     self.get_logger().info(f"Tree {msg.data} selected")

    def get_width(self):
        # retained for compatibility: treat TreeSelect as linear index (1..18)
        sel = int(self.ui.TreeSelect.value())
        msg = Float32()
        msg.data = float(sel)
        self.tree_pub.publish(msg)
        self.get_logger().info(f"Tree {msg.data} selected")

    def on_get_width_clicked(self):
        # ask user for row and column
        row, ok1 = QtWidgets.QInputDialog.getInt(self.window, "Select Row", "Row (1=left,2=center,3=right):", 2, 1, 3, 1)
        if not ok1:
            return
        col, ok2 = QtWidgets.QInputDialog.getInt(self.window, "Select Column", "Column (1..6, bottom->top):", 1, 1, 6, 1)
        if not ok2:
            return
        self.get_width_by_row_col(row, col)

    def get_width_by_row_col(self, row: int, col: int):
        # validate inputs
        if row not in self.row_to_x or col not in self.col_to_y:
            self.ui.label.setText("Width: invalid")
            return
        tx = self.row_to_x[row]
        ty = self.col_to_y[col]

        # try to find measurement in latest_trees by matching x,y
        for entry in self.latest_trees:
            if entry['x'] == tx and entry['y'] == ty:
                w = float(entry['width'])
                self.ui.label.setText(f"Width: {w:.2f} m")
                # publish selected tree index consistent with detector ordering:
                # detector order: center column (x=0) ids 1..6, left column (x=-4) ids 7..12, right column (x=4) ids 13..18
                base = 0
                if row == 1:   # left
                    base = 6
                elif row == 2: # center
                    base = 0
                elif row == 3: # right
                    base = 12
                idx = base + (col - 1) + 1  # 1-based id
                msg = Float32()
                msg.data = float(idx)
                self.tree_pub.publish(msg)
                self.get_logger().info(f"Tree (row={row},col={col}) selected -> id {idx}")
                return
        # not found
        self.ui.label.setText("Width: no data")

    # --- Subscriber callbacks ---
    def status_callback(self, msg):
        self.ui.statusbar.showMessage(msg.data)

    def width_callback(self, msg):
        self.ui.label.setText(f"Width: {msg.data:.2f} m")

    def trees_callback(self, msg: TreeDetectionArray):
        # msg.trees is a sequence of TreeDetection messages (id,x,y,width)
        parsed = []
        for t in msg.trees:
            # use integer width,x,y (rounding)
            parsed.append({'width': int(round(t.width)), 'x': int(round(t.x)), 'y': int(round(t.y))})
        self.latest_trees = parsed

    def run(self):
        sys.exit(self.app.exec_())

def main():
    rclpy.init()
    node = DroneUINode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
