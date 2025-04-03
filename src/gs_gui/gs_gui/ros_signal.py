from PyQt5.QtCore import pyqtSignal, QObject
class ROSSignal(QObject):
    send_command= pyqtSignal(str)  # Signal to send a command