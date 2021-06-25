#!/usr/bin/env python
import os.path
import sys

# PyQt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5 import QtCore
from PyQt5 import QtGui

# ROS
import rospy

from mobile_manipulator import MobileManipulator

rospy.init_node('ur_gui_node', anonymous=True)

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

        # Initialize Buttons
        self.pushButton_sendPoseList.clicked.connect(self.pushButton_sendPoseList_click)

        self.pushButton_processRunStop.setCheckable(True)
        self.pushButton_processRunStop.toggled.connect(self.pushButton_processRunStop_toggle)

        # Initialize Edit Boxes
        self.lineEdit_distance_min.setText("0.10")
        self.lineEdit_distance_max.setText("0.20")
        self.lineEdit_distance_interval.setText("0.05")
        self.lineEdit_pitch_min.setText("-30.0")
        self.lineEdit_pitch_max.setText("30.0")
        self.lineEdit_pitch_interval.setText("10.0")

        # MobileManipulator (Thread Class) Initialization
        self.mobile_manipulator = MobileManipulator()

        # Label Image Initialization
        self.label_image.resize(self.mobile_manipulator.cam.width, self.mobile_manipulator.cam.height)

        # Initialize Timer
        timeout = rospy.Duration(30)
        self.timer=QTimer()
        self.timer.timeout.connect(self.timerEvent)

        # Start Timer
        self.timer.start(100)

    def pushButton_sendPoseList_click(self):

            distance_min = float(self.lineEdit_distance_min.text())
            distance_max = float(self.lineEdit_distance_max.text())
            distance_interval = float(self.lineEdit_distance_interval.text())

            pitch_min = float(self.lineEdit_pitch_min.text())
            pitch_max = float(self.lineEdit_pitch_max.text())
            pitch_interval = float(self.lineEdit_pitch_interval.text())

            self.mobile_manipulator.calculate_pose_list(distance_min, distance_max, distance_interval, pitch_min, pitch_max, pitch_interval)
            self.mobile_manipulator.operation_list = ["MOVE_CARTESIAN"]

    def pushButton_processRunStop_toggle(self, checked):
        if(checked):
            self.mobile_manipulator.isRun = True
            self.mobile_manipulator.start()
            self.label_process.setText("Process: MobileManipulator Thread is Running")
            self.pushButton_processRunStop.setText("Process is Running")

        else:
            self.mobile_manipulator.isRun = False
            self.label_process.setText("Process: MobileManipulator Thread is Stopped")
            self.pushButton_processRunStop.setText("Process is Stopped")

    def timerEvent(self):
        if self.mobile_manipulator.isRun == False:
            if self.pushButton_processRunStop.isChecked() == True:
                self.pushButton_processRunStop.toggle()

        # Robot Mode
        answer = self.mobile_manipulator.ur_state.robot_mode_resp.answer
        resp = self.mobile_manipulator.ur_state.robot_mode_resp
        self.mode = resp.robot_mode.mode
        self.label_robotMode.setText(answer)

        # Program State
        resp = self.mobile_manipulator.ur_state.program_state_resp
        state = resp.state.state
        full_state = "Program state: " + state
        self.label_programState.setText(full_state)

        resp = self.mobile_manipulator.ur_state.loaded_program_resp
        answer = resp.answer
        self.label_loadedProgram.setText(answer)

        resp = self.mobile_manipulator.ur_state.safety_mode_resp
        answer = resp.answer
        self.label_safetyMode.setText(answer)

        if self.mobile_manipulator.ur_state.trans != None:
            trans = self.mobile_manipulator.ur_state.trans
            px = str(trans.transform.translation.x)
            py = str(trans.transform.translation.y)
            pz = str(trans.transform.translation.z)
            rx = str(trans.transform.rotation.x)
            ry = str(trans.transform.rotation.y)
            rz = str(trans.transform.rotation.z)
            rw = str(trans.transform.rotation.w)

            str_pos = "Endeffector Pose: "+ "\n" + "[" + px + "," + "\n" + py + "," + "\n" + pz + "," + "\n" + rx + "," + "\n" + ry + "," + "\n" + rz + "," + "\n" + rw + "]"

            #print(str_pos)

            self.label_endeffectorPos.setText(str_pos)

        # Image Label
        if self.mobile_manipulator.cam.isRun == True:
            h,w,c = self.mobile_manipulator.cam.img.shape
            qImg = QtGui.QImage(self.mobile_manipulator.cam.img.data, w, h, w*c, QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(qImg)
            self.label_image.setPixmap(pixmap)

        # Pose List
        if len(self.mobile_manipulator.pose_list) > 0:
            str_pose_list = ""
            for pose in self.mobile_manipulator.pose_list:
                str_pose_list = str_pose_list + str(pose) + "\n"

            self.label_poseList.setText(str_pose_list)
        else:
            str_pose_list = "There are No Poses"
            self.label_poseList.setText(str_pose_list)




    def closeEvent(self, event):
        reply = QMessageBox.question(self, 'Quit?',
                                     'Are you sure you want to quit?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            if not type(event) == bool:
                event.accept()
                self.mobile_manipulator.ur.finalize()
            else:
                sys.exit()
        else:
            if not type(event) == bool:
                event.ignore()






if __name__ == '__main__':


    app = QApplication(sys.argv)
    ur_gui = UR_GUI()
    ur_gui.show()
    app.exec_()
