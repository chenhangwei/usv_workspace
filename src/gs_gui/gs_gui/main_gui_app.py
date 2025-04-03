import sys
import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QApplication, QMainWindow,QVBoxLayout
from gs_gui.ros_signal import ROSSignal
from gs_gui.ros2_node_for_gui import GroundStationNode
from gs_gui.ui import Ui_MainWindow

class Mainwindow(QMainWindow):
    def __init__(self,ros_signal):
        super().__init__()
        self.ui=Ui_MainWindow()        
        self.ui.setupUi(self)
        self.setWindowTitle("Ground Station GUI")
        self.resize(1024, 512)
        self.setGeometry(100, 100, 1124, 612)

        # 启动 RViz2
        self.rviz_process = QProcess(self)
        self.rviz_process.start("rviz2")

def main(argv=None):
    app = QApplication(sys.argv)
    ros_signal = ROSSignal()

    rclpy.init(args=None)
    node=GroundStationNode(ros_signal)

    ros_signal.send_command.connect(node.publish_command)#主线程连接

    ros_thread = threading.Thread(target=lambda:rclpy.spin(node), daemon=True)
    ros_thread.start()
   
    main_window = Mainwindow(ros_signal)
    main_window.show()
    sys.exit(app.exec_())
    
if __name__ == "__main__":
    main()
