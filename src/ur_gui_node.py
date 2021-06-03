#!/usr/bin/env python
import os.path
import sys

# PyQt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5 import QtCore

# ROS
import rospy
from ur_dashboard_msgs.srv import GetRobotMode, GetProgramState, GetLoadedProgram, GetSafetyMode, Load
from std_srvs.srv import Trigger




class TestThread(QThread):
    # thread custom event
    # gotta name type of data
    threadEvent = QtCore.pyqtSignal(int)

    def __init__(self, parent=None):
        super(TestThread,self).__init__()
        self.n = 0
        self.main = parent
        self.isRun = False

    def run(self):
        while self.isRun:
            print('thread : ' + str(self.n))

            # 'threadEvent' event happened
            # can send parameter (also instance)
            self.threadEvent.emit(self.n)

            self.n += 1
            self.sleep(1)








# Reading UI File
def find_data_file(filename):
    if getattr(sys, 'frozen', False):
        # The application is frozen
        datadir = os.path.dirname(sys.executable)
    else:
        # The application is not frozen
        # Change this bit to match where you store your data files:
        datadir = os.path.dirname(__file__)
    return os.path.join(datadir, filename)

form_class = uic.loadUiType(find_data_file("ur_gui.ui"))[0]

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
        
        self.s_loadProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        self.s_powerOn = rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_on', Trigger)
        self.s_powerOff = rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_off', Trigger)
        self.s_breakRelease = rospy.ServiceProxy('/ur_hardware_interface/dashboard/brake_release', Trigger)
        self.s_playProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        self.s_stopProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)

        self.s_loadProgram("/programs/ros.urp")

        resp = self.s_getRobotMode()
        self.mode = resp.robot_mode.mode
        
        # Initialize Buttons
        self.pushButton_robotRunStop.setCheckable(True)
        self.pushButton_robotRunStop.toggled.connect(self.pushButton_robotRunStop_toggle)

        self.pushButton_processRunStop.setCheckable(True)
        self.pushButton_processRunStop.toggled.connect(self.pushButton_processRunStop_toggle)

        # Start Timer
        self.timer.start(1000)

        # Create Thread Instance
        self.th = TestThread(self)

        # Connect Thread Event
        self.th.threadEvent.connect(self.threadEventHandler)

    @pyqtSlot()
    def threadStart(self):
        if not self.th.isRun:
            print('Main : Begin Thread')
            self.th.isRun = True
            self.th.start()

    @pyqtSlot()
    def threadStop(self):
        if self.th.isRun:
            print('Main : Stop Thread')
            self.th.isRun = False

    @pyqtSlot(int)
    def threadEventHandler(self, n):
        print('Main : threadEvent(self,' + str(n) + ')')




    def pushButton_robotRunStop_toggle(self, checked):
        if(checked):
            # Running the Robot
            self.pushButton_robotRunStop.setText("Stop Robot")
            resp = self.s_powerOn()
            print(resp)
            print("\n")
            rate = rospy.Rate(1)

            while True:
                resp = self.s_getRobotMode()
                self.mode = resp.robot_mode.mode
                #print(self.mode)
                rate.sleep()

                if self.mode == 5:
                    #print("stop")
                    break

            resp = self.s_breakRelease()
            print(resp)
            print("\n")

            while True:
                resp = self.s_getRobotMode()
                self.mode = resp.robot_mode.mode
                #print(self.mode)
                rate.sleep()

                if self.mode == 7:
                    #print("stop")
                    break

            resp = self.s_playProgram()
            print(resp)
            print("\n")


        else:
            # Stop the Robot
            self.pushButton_robotRunStop.setText("Run Robot")
            self.s_powerOff()

            resp = self.s_stopProgram()
            print(resp)
            print("\n")

    def pushButton_processRunStop_toggle(self, checked):
        if(checked):
            # Running the Process
            self.pushButton_processRunStop.setText("Stop Process")

            if not self.th.isRun:
                print('Main : Begin Thread')
                self.th.isRun = True
                self.th.start()
        else:
            # Stop the Process
            self.pushButton_processRunStop.setText("Run Process")

            if self.th.isRun:
                print('Main : Stop Thread')
                self.th.isRun = False

    def timerEvent(self):
        resp = self.s_getRobotMode()
        self.mode = resp.robot_mode.mode
        answer = resp.answer
        self.label_robotMode.setText(answer)

        resp = self.s_getProgramState()
        state = resp.state.state
        state = "Program state: " + state
        self.label_programState.setText(state)

        print(state)
        if state == "Program state: STOPPED":
            resp = self.s_playProgram()
            print(resp)

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
