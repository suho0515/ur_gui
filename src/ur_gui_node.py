#!/usr/bin/env python
import sys

# PyQt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QTimer, QTime

# ROS
import rospy
from ur_dashboard_msgs.srv import GetRobotMode, GetProgramState, GetLoadedProgram, GetSafetyMode

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
        self.s_getRobotMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)
        self.s_getProgramState = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        self.s_getLoadedProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram)
        self.s_getSafetyMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)

        # Start Timer
        self.timer.start(1000)

    def timerEvent(self):
        resp = self.s_getRobotMode()
        mode = resp.robot_mode.mode
        answer = resp.answer
        self.label_robotMode.setText(answer)

        resp = self.s_getProgramState()
        answer = resp.state.state
        answer = "Program state: " + answer
        self.label_programState.setText(answer)

        resp = self.s_getLoadedProgram()
        answer = resp.answer
        self.label_loadedProgram.setText(answer)

        resp = self.s_getSafetyMode()
        answer = resp.answer
        self.label_safetyMode.setText(answer)



if __name__ == '__main__':


    app = QApplication(sys.argv)
    ur_gui = UR_GUI()
    ur_gui.show()
    app.exec_()
