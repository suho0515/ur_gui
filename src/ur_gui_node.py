#!/usr/bin/env python
import os.path
import sys
import unittest

# PyQt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5 import QtCore

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


    def __init__(self, parent=None):
        super(UR_Thread,self).__init__()
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

        self.s_getRobotMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)
        self.s_getProgramState = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        self.s_getLoadedProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram)
        self.s_getSafetyMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)
        
        self.s_loadProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        self.s_loadProgram("/programs/ros.urp")

        self.s_playProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)

        resp = self.s_getRobotMode()
        self.mode = resp.robot_mode.mode
        
        #---
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


        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)


        # Initialize Buttons
        self.pushButton_robotRunStop.setCheckable(True)
        self.pushButton_robotRunStop.toggled.connect(self.pushButton_robotRunStop_toggle)

        self.pushButton_processRunStop.setCheckable(True)
        self.pushButton_processRunStop.toggled.connect(self.pushButton_processRunStop_toggle)


        # Initialize Edit Boxes
        self.lineEdit_z_min.setText("0.10")
        self.lineEdit_z_max.setText("0.20")
        self.lineEdit_z_interval.setText("0.05")
        self.lineEdit_pitch_min.setText("-30.0")
        self.lineEdit_pitch_max.setText("30.0")
        self.lineEdit_pitch_interval.setText("10.0")


        # Start Timer
        self.timer.start(1000)

        # Create Thread Instance
        self.th = UR_Thread(self)

        # Connect Thread Event
        self.th.threadEvent.connect(self.threadEventHandler)



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

    def move_cartesian(self,x,y,z,rx,ry,rz):

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

        point.time_from_start = rospy.Duration(1.0)

        goal.trajectory.points.append(point)

        goal.goal_time_tolerance = rospy.Duration(0.6)

        self.cartesian_trajectory_client.send_goal(goal)

        self.cartesian_trajectory_client.wait_for_result()
        print(self.cartesian_trajectory_client.get_result())

        rospy.loginfo("Received result SUCCESSFUL")

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

        z_min = float(self.lineEdit_z_min.text())
        z_max = float(self.lineEdit_z_max.text())
        z_interval = float(self.lineEdit_z_interval.text())

        pitch_min = float(self.lineEdit_pitch_min.text())
        pitch_max = float(self.lineEdit_pitch_max.text())
        pitch_interval = float(self.lineEdit_pitch_interval.text())

        #print (z_max-z_min)
        #print (z_max-z_min)/z_interval

        z_list=[]
        z_list.append(z_min)

        for i in range(int((z_max-z_min)/z_interval)):
            z_list.append(z_min+((i+1)*z_interval))

        print(z_list)

        pitch_list=[]
        pitch_list.append(pitch_min)

        for i in range(int((pitch_max-pitch_min)/pitch_interval)):
            pitch_list.append(pitch_min+((i+1)*pitch_interval))

        print(pitch_list)

        for i in z_list:
            for j in pitch_list:
                print(i)
                print(j)
                self.move_cartesian(0.0,0.3,i,180.0,j,0.0)


        #self.move_cartesian(0.0,0.3,0.2,180.0,0.0,0.0)

        #self.move_cartesian(0.0,0.3,0.1,180.0,0.0,0.0)

        #self.script_publisher.publish("movej([1, -1.7, -1.7, -1, -1.57, -2])")
        '''
        self.switch_on_controller("pose_based_cartesian_traj_controller")

        goal = FollowCartesianTrajectoryGoal()

        point = CartesianTrajectoryPoint()


        point.pose.position.x = 0.0 # red line      0.2   0.2
        point.pose.position.y = 0.300  # green line  0.15   0.15
        #if(z > 0.32):
        point.pose.position.z = 0.200  # blue line   # 0.35   0.6
        #elif (z < 0.31):
        #    point.pose.position.z = 0.33

        rot = Rotation.from_euler('xyz', [180.0, 0.0, 0.0], degrees=True)

        rot_quat = rot.as_quat()
        #print(rot_quat)

        point.pose.orientation.x = rot_quat[0]
        point.pose.orientation.y = rot_quat[1]
        point.pose.orientation.z = rot_quat[2]
        point.pose.orientation.w = rot_quat[3]

        point.time_from_start = rospy.Duration(10.0)

        goal.trajectory.points.append(point)

        point.pose.position.z = 0.200  # blue line   # 0.35   0.6

        goal.trajectory.points.append(point)

        goal.goal_time_tolerance = rospy.Duration(0.6)

        self.cartesian_trajectory_client.send_goal(goal)


        print(self.cartesian_trajectory_client.get_result())

        rospy.loginfo("Received result SUCCESSFUL")
        '''








    def pushButton_robotRunStop_toggle(self, checked):
        # Running the Robot
        if(checked):
            self.set_robot_to_mode(RobotMode.POWER_OFF)
            rospy.sleep(0.5)
            self.set_robot_to_mode(RobotMode.RUNNING)
            rospy.sleep(0.5)

            self.pushButton_robotRunStop.setText("Stop Robot")

        # Stop the Robot
        else:
            self.set_robot_to_mode(RobotMode.POWER_OFF)
            rospy.sleep(0.5)

            self.pushButton_robotRunStop.setText("Run Robot")

    def pushButton_processRunStop_toggle(self, checked):
        # Running the Process
        if(checked):

            self.pushButton_processRunStop.setText("Stop Process")

            if not self.th.isRun:
                print('Main : Begin Thread')
                self.label_process.setText("Process: UR Thread is Running")
                self.th.isRun = True
                self.th.start()

        # Stop the Process
        else:

            self.pushButton_processRunStop.setText("Run Process")

            if self.th.isRun:
                print('Main : Stop Thread')
                self.label_process.setText("Process: UR Thread is Stop")
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












if __name__ == '__main__':


    app = QApplication(sys.argv)
    ur_gui = UR_GUI()
    ur_gui.show()
    app.exec_()
