#!/usr/bin/env python
import os.path
import sys
import unittest

# PyQt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5 import QtCore
from PyQt5 import QtGui

# ROS
import rospy
from ur_dashboard_msgs.srv import GetRobotMode, GetProgramState, GetLoadedProgram, GetSafetyMode, Load
from std_srvs.srv import Trigger


from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, CartesianTrajectoryPoint
import actionlib
from ur_msgs.msg import RobotStateRTMsg, IOStates
from ur_dashboard_msgs.msg import SetModeAction, SetModeGoal, RobotMode
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
import std_msgs.msg
from scipy.spatial.transform import Rotation
import tf2_ros

from math import *

# opencv
import cv2
import time

from cam import CAM

rospy.init_node('ur_gui_node', anonymous=True)

ALL_CONTROLLERS = [
        "scaled_pos_joint_traj_controller",
        "pos_joint_traj_controller",
        "scaled_vel_joint_traj_controller",
        "vel_joint_traj_controller",
        "joint_group_vel_controller",
        "forward_joint_traj_controller",
        "forward_cartesian_traj_controller",
        "twist_controller",
        "pose_based_cartesian_traj_controller",
        "joint_based_cartesian_traj_controller",
        ]

class UR_Thread(QThread):
    # thread custom event
    # gotta name type of data
    threadEvent = QtCore.pyqtSignal(int)


    def __init__(self, parent=None, ur_gui=None, cam_th=None):
        super(UR_Thread,self).__init__()
        self.n = 0
        self.main = parent
        self.isRun = False
        self.ur_gui = ur_gui
        self.cam_th = cam_th

        # UR Control
        timeout = rospy.Duration(30)

        self.set_mode_client = actionlib.SimpleActionClient(
            '/ur_hardware_interface/set_mode', SetModeAction)
        try:
            self.set_mode_client.wait_for_server(timeout)
            print("set mode action client is on")
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.cartesian_trajectory_client = actionlib.SimpleActionClient(
            '/pose_based_cartesian_traj_controller/follow_cartesian_trajectory', FollowCartesianTrajectoryAction)
        try:
            self.cartesian_trajectory_client.wait_for_server(timeout)
            print("follow cartesian trajectory action client is on")
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach cartesian controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.switch_controllers_client = rospy.ServiceProxy('/controller_manager/switch_controller',
                SwitchController)
        try:
            self.switch_controllers_client.wait_for_service(timeout)
            print("controller switch service is on")
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach controller switch service. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.script_publisher = rospy.Publisher("/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1)

    def set_robot_to_mode(self, target_mode):
        goal = SetModeGoal()
        goal.target_robot_mode = target_mode
        goal.play_program = True # we use headless mode during tests
        # This might be a bug to hunt down. We have to reset the program before calling `resend_robot_program`
        goal.stop_program = False

        self.set_mode_client.send_goal(goal)
        self.set_mode_client.wait_for_result()
        return self.set_mode_client.get_result().success


    def switch_on_controller(self, controller_name):
        """Switches on the given controller stopping all other known controllers with best_effort
        strategy."""
        srv = SwitchControllerRequest()
        srv.stop_controllers = ALL_CONTROLLERS
        srv.start_controllers = [controller_name]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = self.switch_controllers_client(srv)
        print(result)

    def move_cartesian(self,x,y,z,rx,ry,rz,long_movement):

        goal = FollowCartesianTrajectoryGoal()

        point = CartesianTrajectoryPoint()


        point.pose.position.x = x # red line      0.2   0.2
        point.pose.position.y = y  # green line  0.15   0.15
        point.pose.position.z = z  # blue line   # 0.35   0.6

        rot = Rotation.from_euler('xyz', [rx, ry, rz], degrees=True)
        rot_quat = rot.as_quat()
        #print(rot_quat)

        point.pose.orientation.x = rot_quat[0]
        point.pose.orientation.y = rot_quat[1]
        point.pose.orientation.z = rot_quat[2]
        point.pose.orientation.w = rot_quat[3]

        if long_movement:
            point.time_from_start = rospy.Duration(5.0)
        else:
            point.time_from_start = rospy.Duration(1.0)

        goal.trajectory.points.append(point)

        goal.goal_time_tolerance = rospy.Duration(0.6)

        self.cartesian_trajectory_client.send_goal(goal)

        self.cartesian_trajectory_client.wait_for_result()
        #print(self.cartesian_trajectory_client.get_result())

        rospy.loginfo("Received result SUCCESSFUL")



    def run(self):
        #while self.isRun:
        print('thread : ' + str(self.n))

        self.set_robot_to_mode(RobotMode.POWER_OFF)
        rospy.sleep(0.5)
        self.set_robot_to_mode(RobotMode.RUNNING)
        rospy.sleep(0.5)

        state = ""
        while not state == "PLAYING":
            rospy.wait_for_service('/ur_hardware_interface/dashboard/program_state')
            resp = self.ur_gui.s_getProgramState()
            state = resp.state.state
            full_state = "Program state: " + state
            print(state)
            self.ur_gui.label_programState.setText(full_state)


            # there is some bug that the program stop once when it play.
            # so i set the watchdog that makes program run again when it fails.
            # print(state)
            rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
            resp = self.ur_gui.s_playProgram()

            rospy.sleep(1)

            print(resp)
            if resp.success == True:
                #rospy.sleep(10)
                break



        distance_min = float(self.ur_gui.lineEdit_distance_min.text())
        distance_max = float(self.ur_gui.lineEdit_distance_max.text())
        distance_interval = float(self.ur_gui.lineEdit_distance_interval.text())

        pitch_min = float(self.ur_gui.lineEdit_pitch_min.text())
        pitch_max = float(self.ur_gui.lineEdit_pitch_max.text())
        pitch_interval = float(self.ur_gui.lineEdit_pitch_interval.text())

        #print (distance_max-distance_min)
        #print (distance_max-distance_min)/distance_interval

        distance_list=[]
        distance_list.append(distance_min)

        for i in range(int((distance_max-distance_min)/distance_interval)):
            distance_list.append(distance_min+((i+1)*distance_interval))

        print(distance_list)

        pitch_list=[]
        pitch_list.append(pitch_min)

        for i in range(int((pitch_max-pitch_min)/pitch_interval)):
            pitch_list.append(pitch_min+((i+1)*pitch_interval))

        print(pitch_list)

        self.move_cartesian(0.0,0.3,0.1,180.0,0.0,90.0,long_movement=True)
        num = 0
        for distance in distance_list:
            long_movement = True

            for pitch in pitch_list:

                print(distance)
                print(pitch)
                print(radians(pitch))
                x = distance * cos(radians(90-pitch))
                z = distance * sin(radians(90-pitch))
                print(x)
                print(z)
                print("\n")

                self.move_cartesian(x,0.3,z+0.04,180.0+pitch,0.0,90.0,long_movement)

                long_movement = False

                self.cam_th.save_image(num)
                num = num + 1

        self.move_cartesian(0.0,0.3,0.1,180.0,0.0,90.0,long_movement=True)

        self.ur_gui.pushButton_processRunStop.setText("Run Process")
        self.ur_gui.label_process.setText("Process: UR Thread is Stoped")

        # 'threadEvent' event happened
        # can send parameter (also instance)
        #self.threadEvent.emit(self.n)

        #self.n += 1
        #self.sleep(1)

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

        self.s_getRobotMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)
        self.s_getProgramState = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        self.s_getLoadedProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram)
        self.s_getSafetyMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)
        
        self.s_loadProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        self.s_loadProgram("/programs/ros.urp")

        self.s_playProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)

        resp = self.s_getRobotMode()
        self.mode = resp.robot_mode.mode

        timeout = rospy.Duration(30)

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)


        # Initialize Buttons
        self.pushButton_robotRunStop.setCheckable(True)
        self.pushButton_robotRunStop.toggled.connect(self.pushButton_robotRunStop_toggle)

        #self.pushButton_processRunStop.setCheckable(True)
        self.pushButton_processRunStop.clicked.connect(self.pushButton_processRunStop_clicked)


        # Initialize Edit Boxes
        self.lineEdit_distance_min.setText("0.10")
        self.lineEdit_distance_max.setText("0.20")
        self.lineEdit_distance_interval.setText("0.05")
        self.lineEdit_pitch_min.setText("-30.0")
        self.lineEdit_pitch_max.setText("30.0")
        self.lineEdit_pitch_interval.setText("10.0")

        # Camera Sensor (Thread Class) Initialization
        self.cam = CAM()

        if not self.cam.isRun:
            print('Main : Begin CAM Thread')
            self.cam.isRun = True
            self.cam.start()

        # Label Image Initialization
        self.label_image.resize(self.cam.width, self.cam.height)

        # Create UR Thread Instance
        #self.th = UR_Thread(self, ur_gui=self, cam_th=self.cam_th)

        # Connect UR Thread Event
        #self.th.threadEvent.connect(self.threadEventHandler)

        # Start Timer
        self.timer.start(100)



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
        # Thread works only once
        self.threadStop()




    def pushButton_robotRunStop_toggle(self, checked):
        # Running the Robot
        if(checked):
            self.th.set_robot_to_mode(RobotMode.POWER_OFF)
            rospy.sleep(0.5)
            self.th.set_robot_to_mode(RobotMode.RUNNING)
            rospy.sleep(0.5)

            self.pushButton_robotRunStop.setText("Stop Robot")

        # Stop the Robot
        else:
            self.th.set_robot_to_mode(RobotMode.POWER_OFF)
            rospy.sleep(0.5)

            self.pushButton_robotRunStop.setText("Run Robot")

    def pushButton_processRunStop_clicked(self):

        if not self.th.isRun:
            self.pushButton_processRunStop.setText("Process is Running")
            print('Main : Begin Thread')
            self.label_process.setText("Process: UR Thread is Running")
            self.th.isRun = True
            self.th.start()



    def timerEvent(self):
        resp = self.s_getRobotMode()
        self.mode = resp.robot_mode.mode
        answer = resp.answer
        self.label_robotMode.setText(answer)

        resp = self.s_getProgramState()
        state = resp.state.state
        state = "Program state: " + state
        self.label_programState.setText(state)

        # there is some bug that the program stop once when it play.
        # so i set the watchdog that makes program run again when it fails.
        # print(state)
        if state == "Program state: STOPPED":
            resp = self.s_playProgram()
            #print(resp)

        resp = self.s_getLoadedProgram()
        answer = resp.answer
        self.label_loadedProgram.setText(answer)

        resp = self.s_getSafetyMode()
        answer = resp.answer
        self.label_safetyMode.setText(answer)

        trans = self.tfBuffer.lookup_transform('base', 'tool0_controller', rospy.Time())
        #print(trans)
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
        if self.cam.isRun == True:
            h,w,c = self.cam.img.shape
            qImg = QtGui.QImage(self.cam.img.data, w, h, w*c, QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(qImg)
            self.label_image.setPixmap(pixmap)




    def closeEvent(self, event):
        reply = QMessageBox.question(self, 'Quit?',
                                     'Are you sure you want to quit?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            if not type(event) == bool:
                event.accept()
                self.th.set_robot_to_mode(RobotMode.POWER_OFF)
                rospy.sleep(0.5)
                print "test"
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
