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

# Moveit
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, CartesianTrajectoryPoint
import actionlib
from ur_msgs.msg import RobotStateRTMsg, IOStates
from ur_dashboard_msgs.msg import SetModeAction, SetModeGoal, RobotMode
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
import std_msgs.msg
from scipy.spatial.transform import Rotation


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


    #moveit_commander.roscpp_initialize(sys.argv)
    #robot = moveit_commander.RobotCommander()
    #scene = moveit_commander.PlanningSceneInterface()
    #move_group = moveit_commander.MoveGroupCommander("manipulator")

    #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                               moveit_msgs.msg.DisplayTrajectory,
    #                                               queue_size=20)

    # We can get the name of the reference frame for this robot:
    #planning_frame = move_group.get_planning_frame()
    #print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    #eef_link = move_group.get_end_effector_link()
    #print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    #group_names = robot.get_group_names()
    #print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""

    #print "============ Printing robot state"
    #print move_group.get_current_pose()
    #print ""



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





        # Start Timer
        self.timer.start(1000)

        # Create Thread Instance
        self.th = UR_Thread(self)

        # Connect Thread Event
        self.th.threadEvent.connect(self.threadEventHandler)

    def fb_callback(feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))

    def switch_on_controller(self, controller_name):
        """Switches on the given controller stopping all other known controllers with best_effort
        strategy."""
        srv = SwitchControllerRequest()
        srv.stop_controllers = ALL_CONTROLLERS
        srv.start_controllers = [controller_name]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = self.switch_controllers_client(srv)
        print(result)


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

        self.threadStop()

        #self.script_publisher.publish("movej([1, -1.7, -1.7, -1, -1.57, -2])")

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
        print(rot_quat)


        point.pose.orientation.x = rot_quat[0]
        point.pose.orientation.y = rot_quat[1]
        point.pose.orientation.z = rot_quat[2]
        point.pose.orientation.w = rot_quat[3]

        point.time_from_start = rospy.Duration(10.0)
        goal.trajectory.points.append(point)
        goal.goal_time_tolerance = rospy.Duration(0.6)

        self.cartesian_trajectory_client.send_goal(goal)


        self.cartesian_trajectory_client.wait_for_result()
        #print(self.cartesian_trajectory_client.get_result())

        rospy.loginfo("Received result SUCCESSFUL")












        #print "============ Printing robot state"
        #print self.th.move_group.get_current_pose()
        #print ""

        #z = self.th.move_group.get_current_pose().pose.position.z



        #pose_goal = geometry_msgs.msg.Pose()
        #pose_goal.orientation.x = -0.741903983597
        #pose_goal.orientation.y = -0.0167735786841
        #pose_goal.orientation.z = 0.668109844767
        #pose_goal.orientation.w = 0.0540958547871
        #pose_goal.position.x = -0.122104521697 # red line      0.2   0.2
        #pose_goal.position.y = -0.247255641221  # green line  0.15   0.15
        #if(z > 0.32):
        #    pose_goal.position.z = 0.30  # blue line   # 0.35   0.6
        #elif (z < 0.31):
        #    pose_goal.position.z = 0.33
        #self.th.move_group.set_pose_target(pose_goal)

        #plan = self.th.move_group.go(wait=True)
        #move_group.stop()
        #move_group.clear_pose_targets()

        #rospy.sleep(2)




    def pushButton_robotRunStop_toggle(self, checked):
        if(checked):
            # Running the Robot
            self.pushButton_robotRunStop.setText("Stop Robot")
            resp = self.s_powerOn()
            print(resp)
            print("\n")
            # there is certain error about Rate() Function.
            # I should fix it later.
            #rate = rospy.Rate(1)

            while True:
                resp = self.s_getRobotMode()
                self.mode = resp.robot_mode.mode
                print(self.mode)
                #rate.sleep()
                if self.mode == 5:
                    print("stop")
                    break

            resp = self.s_breakRelease()
            print(resp)
            print("\n")

            while True:
                resp = self.s_getRobotMode()
                self.mode = resp.robot_mode.mode
                print(self.mode)
                #rate.sleep()

                if self.mode == 7:
                    print("stop")
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
                self.label_process.setText("Process: UR Thread is Running")
                self.th.isRun = True
                self.th.start()
        else:
            # Stop the Process
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

        #px = str(self.th.move_group.get_current_pose().pose.position.x)
        #py = str(self.th.move_group.get_current_pose().pose.position.y)
        #pz = str(self.th.move_group.get_current_pose().pose.position.z)
        #ox = str(self.th.move_group.get_current_pose().pose.orientation.x)
        #oy = str(self.th.move_group.get_current_pose().pose.orientation.y)
        #oz = str(self.th.move_group.get_current_pose().pose.orientation.z)
        #ow = str(self.th.move_group.get_current_pose().pose.orientation.w)

        #str_pos = "Endeffector Pose: " + "[" + px + "," + py + "," + pz + "," + ox + "," + oy + "," + oz + "," + ow + "]"

        #print(str_pos)

        #self.label_endeffectorPos.setText(str_pos)












if __name__ == '__main__':


    app = QApplication(sys.argv)
    ur_gui = UR_GUI()
    ur_gui.show()
    app.exec_()
