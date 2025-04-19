import sys
import threading

from sympy import Point
import rclpy
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QTableWidgetItem
from PyQt5.QtGui import QStandardItemModel, QStandardItem
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

        self.ros_signal.receive_state_list.connect(self.receive_state_callback)  # 连接信号

        self.ui.arming_pushButton.clicked.connect(self.cluster_arming_command)            # 集群解锁按钮
        self.ui.disarming_pushButton.clicked.connect(self.cluster_disarming_command)      # 集群加锁按钮
        self.ui.set_guided_pushButton.clicked.connect(self.set_cluster_guided_command)    # 集群切换到guided模式
        self.ui.set_manual_pushButton.clicked.connect(self.set_cluster_manaul_command)    # 集群切换到manaul模式

        self.ui.departed_arming_pushButton.clicked.connect(self.departed_arming_command)  # 离群解锁按钮
        self.ui.departed_disarming_pushButton.clicked.connect(self.departed_disarming_command) #离群加锁按钮
        self.ui.set_departed_guided_pushButton.clicked.connect(self.set_departed_guided_command) #离群切换到guided模式
        self.ui.set_departed_manual_pushButton.clicked.connect(self.set_departed_manaul_command) #离群切换到manual模式

        self.ui.send_cluster_point_pushButton.clicked.connect(self.send_cluster_point_command)
        self.ui.usv_cluster_velocity_horizontalSlider.valueChanged.connect(self.cluster_velocity_value_changed)

        self.ui.send_departed_point_pushButton.clicked.connect(self.send_departed_point_command)
        self.ui.usv_departed_velocity_horizontalSlider.valueChanged.connect(self.departed_velocity_value_changed)

        self.ui.add_cluster_pushButton.connect(self.add_cluster_command)
        self.ui.quit_cluster_pushButton.connect(self.quit_cluster_command)


        #在线设备列表
        self.usv_online_list=[]
        #集群设备列表
        self.usv_cluster_list=[]
        #离群设备列表
        self.usv_departed_list=[]
      

        self.table_model=QStandardItemModel(self)

        self.ui.cluster_tableView.setModel(self.table_model)
        # 启动 RViz2
        self.rviz_process = QProcess(self)
        self.rviz_process.start("rviz2")

    #集群解锁命令
    def cluster_arming_command(self):   
        self.ros_signal.arm_command.emit(self.usv_cluster_list)

    #集群设置manaul模式命令
    def set_cluster_manaul_command(self):      
        self.ros_signal.mode_command.emit(self.usv_cluster_list)

    #集群加锁命令  
    def cluster_disarming_command(self):
        self.ros_signal.disarm_command.emit(self.usv_cluster_list)

    #集群设置guided模式命令 
    def set_cluster_guided_command(self):
        self.ros_signal.guided_command.emit(self.usv_cluster_list)

    #离群解锁命令
    def departed_arming_command (self):        
         self.ros_signal.arm_command.emit(self.usv_departed_list)

    #离群解锁命令
    def departed_disarming_command (self):        
         self.ros_signal.disarm_command.emit(self.usv_departed_list)     
         
    #离群设置manaul模式命令
    def set_departed_manaul_command (self):        
         self.ros_signal.mode_command.emit(self.usv_departed_list)  

    #离群设置guided模式命令
    def set_departed_guided_command (self):        
         self.ros_signal.guided_command.emit(self.usv_departed_list)   


    # 集群目标点命令
    def send_cluster_point_command(self):       
        x = self.ui.set_cluster_x_doubleSpinBox.value()
        y = self.ui.set_cluster_y_doubleSpinBox.value() 
        target_point = Point(x, y)
        self.ros_signal.cluster_target_point_command.emit(target_point)

    # 集群速度命令
    def cluster_velocity_value_changed(self):
        cluster_velocity=self.ui.usv_cluster_velocity_horizontalSlider.value()
        self.ros_signal.cluster_target_velocity_command.emit(cluster_velocity)

    # 离群目标点命令
    def send_departed_point_command(self):
        x = self.ui.set_departed_x_doubleSpinBox.value()
        y = self.ui.set_departed_y_doubleSpinBox.value() 
        target_point = Point(x, y)
        self.ros_signal.departed_target_point_command.emit(target_point)
    # 离群速度命令 
    def departed_velocity_value_changed (self):
        departed_velocity=self.ui.usv_departed_velocity_horizontalSlider.value()
        self.ros_signal.departed_target_velocity_command.emit(departed_velocity)


    #接收usv状态
    def receive_state_callback(self, msg):
        
        if isinstance(msg, list):

            self.usv_online_list=msg

            # 处理接收到的状态消息
            self.ui.info_textEdit.setText(f"更新在线状态消息")
          



            self.table_model.setRowCount(len(msg))
            self.table_model.setColumnCount(4)
            self.table_model.setHorizontalHeaderLabels(["编号", "时间戳", "坐标格式", "当前模式", 
                                                                 "连接状态", "武装状态"])

            for i, state in enumerate(msg):
                self.table_model.setItem(i, 0, QStandardItem(str(state.get_namespace())))
                self.table_model.setItem(i, 1, QStandardItem(str(state.header.stamp)))
                self.table_model.setItem(i, 2, QStandardItem(str(state.header.frame_id)))
                self.table_model.setItem(i, 3, QStandardItem(str(state.mode)))
                self.table_model.setItem(i, 4, QStandardItem(str(state.connected)))
                self.table_model.setItem(i, 5, QStandardItem(str(state.armed)))
                self.table_model.setItem(i, 6, QStandardItem(str(state.battery_voltage)))
                self.table_model.setItem(i, 7, QStandardItem(str(state.battery_percentage)))
                self.table_model.setItem(i, 8, QStandardItem(str(state.power_supply_status)))
                self.table_model.setItem(i, 9, QStandardItem(str(state.position)))
                self.table_model.setItem(i, 10, QStandardItem(str(state.velocity)))
                self.table_model.setItem(i, 11, QStandardItem(str(state.position)))
                self.table_model.setItem(i, 12, QStandardItem(str(state.yaw)))
        #self.ui.setupUi(self)
  

def main(argv=None):
    app = QApplication(sys.argv)
    ros_signal = ROSSignal()
    main_window = Mainwindow(ros_signal)

    rclpy.init(args=None)
    node=GroundStationNode(ros_signal)


    ros_signal.manaul_command.connect(node.set_manaul_callback)#主线程连接
    ros_signal.guided_command.connect(node.set_guided_callback)
    ros_signal.arm_command.connect(node.set_arming_callback)#主线程连接
    ros_signal.disarm_command.connect(node.set_disarming_callback)

    ros_thread = threading.Thread(target=lambda:rclpy.spin(node), daemon=True)
    ros_thread.start()
   
    
    main_window.show()
    sys.exit(app.exec_())
    
if __name__ == "__main__":
    main()
