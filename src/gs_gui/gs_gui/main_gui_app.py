import sys
import threading
import rclpy
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QTableWidgetItem
from PyQt5.QtGui import QStandardItemModel
from gs_gui.ros_signal import ROSSignal
from gs_gui.ros2_node_for_gui import GroundStationNode
from gs_gui.ui import Ui_MainWindow
from mavros_msgs.msg import State


class Mainwindow(QMainWindow):
    def __init__(self,ros_signal):
        super().__init__()
        self.ui=Ui_MainWindow()        
        self.ui.setupUi(self)
        self.setWindowTitle("Ground Station GUI")
        self.resize(1024, 512)
        self.setGeometry(100, 100, 1124, 612)

        self.ros_signal = ros_signal

        ros_signal.receive_state_list.connect(self.receive_state_callback)  # 连接信号
        self.ui.arm_comboBox.currentTextChanged.connect(self.arm_changeEvent)
        self.ui.cluster_setmode_comboBox.currentTextChanged.connect(self.cluster_setmode_changeEvent)

        self.table_model=QStandardItemModel(self)
        self.ui.cluster_tableView.setModel(self.table_model)
        # 启动 RViz2
        self.rviz_process = QProcess(self)
        self.rviz_process.start("rviz2")
      
    def arm_changeEvent(self):
        # 获取选中的项
        selected_item = self.ui.arm_comboBox.currentText()
        # 发送信号到 ROS 节点
        self.ros_signal.arm_command.emit(selected_item)
        print(f"发送命令: {selected_item}")


    def cluster_setmode_changeEvent(self):
        # 获取选中的项
        selected_item = self.ui.cluster_setmode_comboBox.currentText()
        # 发送信号到 ROS 节点
        self.ros_signal.mode_command.emit(selected_item)
        print(f"发送命令: {selected_item}")    

    def receive_state_callback(self, msg):

        if isinstance(msg, list):
            # 处理接收到的状态消息
            self.ui.info_textEdit.setText(f"更新在线状态消息")
          



            self.table_model.setRowCount(len(msg))
            self.table_model.setColumnCount(4)
            self.table_model.setHorizontalHeaderLabels(["编号", "时间戳", "坐标格式", "当前模式", 
                                                                 "连接状态", "武装状态"])

            for i, state in enumerate(msg):
                self.table_model.setItem(i, 0, QTableWidgetItem(state.get_namespace()))
                self.table_model.setItem(i, 1, QTableWidgetItem(state.header.stamp))
                self.table_model.setItem(i, 2, QTableWidgetItem(state.header.frame_id))
                self.table_model.setItem(i, 3, QTableWidgetItem(state.mode))
                self.table_model.setItem(i, 4, QTableWidgetItem(state.connected))
                self.table_model.setItem(i, 5, QTableWidgetItem(state.armed))
                self.table_model.setItem(i, 6, QTableWidgetItem(state.battery_voltage))
                self.table_model.setItem(i, 7, QTableWidgetItem(state.battery_percentage))
                self.table_model.setItem(i, 8, QTableWidgetItem(state.power_supply_status))
                self.table_model.setItem(i, 9, QTableWidgetItem(state.position))
                self.table_model.setItem(i, 10, QTableWidgetItem(state.velocity))
                self.table_model.setItem(i, 11, QTableWidgetItem(state.position))
                self.table_model.setItem(i, 12, QTableWidgetItem(state.yaw))
        #self.ui.setupUi(self)
  

def main(argv=None):
    app = QApplication(sys.argv)
    ros_signal = ROSSignal()
    main_window = Mainwindow(ros_signal)

    rclpy.init(args=None)
    node=GroundStationNode(ros_signal)


    ros_signal.mode_command.connect(node.set_mode_callback)#主线程连接
    ros_signal.arm_command.connect(node.set_arming_callback)#主线程连接

    ros_thread = threading.Thread(target=lambda:rclpy.spin(node), daemon=True)
    ros_thread.start()
   
    
    main_window.show()
    sys.exit(app.exec_())
    
if __name__ == "__main__":
    main()
