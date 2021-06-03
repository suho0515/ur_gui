#!/usr/bin/env python
import sys

# PyQt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QTimer, QTime

# ROS
import rospy
from ur_dashboard_msgs.srv import GetRobotMode,GetRobotModeResponse

form_class = uic.loadUiType("ur_gui.ui")[0]

class UR_GUI(QMainWindow, form_class) :
    def __init__(self) :
        # Initialize GUI
        super(UR_GUI,self).__init__()
        self.setupUi(self)
        self.setWindowTitle('UR_GUI')

        # Initialize Timer
        self.timer=QTimer()
        self.timer.timeout.connect(self.timerEvent)


        # Initialize ROS
        rospy.init_node('ur_gui_node', anonymous=True)
        self.s = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)

        # Start Timer
        self.timer.start(1000)

    def timerEvent(self):
        resp = self.s()
        mode = resp.robot_mode.mode
        answer = resp.answer
        self.label_robotMode.setText(answer)


if __name__ == '__main__':


    app = QApplication(sys.argv)
    ur_gui = UR_GUI()
    ur_gui.show()
    app.exec_()
