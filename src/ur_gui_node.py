#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
import rospy

form_class = uic.loadUiType("ur_gui.ui")[0]

class UR_GUI(QMainWindow, form_class) :
    def __init__(self) :
        super(UR_GUI,self).__init__()
        self.setupUi(self)

        self.label_robotMode.setText("This is Label - Change Text")


if __name__ == '__main__':
    rospy.init_node('ros_node', anonymous=True)

    app = QApplication(sys.argv)
    ur_gui = UR_GUI()
    ur_gui.show()
    app.exec_()
