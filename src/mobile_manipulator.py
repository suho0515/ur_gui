#QThread
from PyQt5.QtCore import QThread

# ROS
import rospy
import actionlib
from std_srvs.srv import Trigger
import std_msgs.msg
from std_msgs.msg import Bool

# UR
from ur_dashboard_msgs.msg import SetModeAction, \
                                    SetModeGoal, \
                                    RobotMode
from ur_dashboard_msgs.srv import GetRobotMode, \
                                    GetProgramState, \
                                    GetLoadedProgram, \
                                    GetSafetyMode, \
                                    Load
from controller_manager_msgs.srv import SwitchControllerRequest, \
                                        SwitchController
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, \
                                        FollowCartesianTrajectoryGoal, \
                                        CartesianTrajectoryPoint

# Python Mathmatics Modules
from scipy.spatial.transform import Rotation
from math import *

from cam import CAM
from ur import UR, URState



class MobileManipulator(QThread):
    def __init__(self, parent=None):
        super(MobileManipulator,self).__init__()
        self.main = parent
        self.isRun = False

        print("Mobile Manipulator Class is Initializing")

        self.pose_list = []
        self.operation = None
        self.operation_list = []
        self.num = 0

        # Camera (Thread)
        self.cam = CAM()

        if not self.cam.isRun:
            print('Main : Begin CAM Thread')
            self.cam.isRun = True
            self.cam.start()

        # UR
        self.ur = UR()
        rospy.sleep(5)

        self.ur_state = URState()
        if not self.ur_state.isRun:
            print('Main : Begin URState Thread')
            self.ur_state.isRun = True
            self.ur_state.start()

    def calculate_pose_list(self, distance_min, distance_max, distance_interval, pitch_min, pitch_max, pitch_interval):
        distance_list=[]
        distance_list.append(distance_min)

        for i in range(int((distance_max-distance_min)/distance_interval)):
            distance_list.append(distance_min+((i+1)*distance_interval))

        #print(distance_list)

        pitch_list=[]
        pitch_list.append(pitch_min)

        for i in range(int((pitch_max-pitch_min)/pitch_interval)):
            pitch_list.append(pitch_min+((i+1)*pitch_interval))

        #print(pitch_list)

        from_start_time = 5.0
        pose = [0.0,0.3,0.1,180.0,0.0,90.0,from_start_time]
        self.pose_list.append(pose)

        for distance in distance_list:
            from_start_time = 5.0
            for pitch in pitch_list:

                #print(distance)
                #print(pitch)
                #print(radians(pitch))
                x = distance * cos(radians(90-pitch))
                z = distance * sin(radians(90-pitch))
                #print(x)
                #print(z)
                #print("\n")

                pose = [x,0.3,z+0.04,180.0+pitch,0.0,90.0,from_start_time]
                self.pose_list.append(pose)

                from_start_time = 1.0

        from_start_time = 5.0
        pose = [0.0,0.3,0.1,180.0,0.0,90.0,from_start_time]
        self.pose_list.append(pose)

    def run(self):
        while self.isRun:
            #print(self.operation_list == None)
            if len(self.operation_list) != 0:
                print "self.operation_list[0]: " + str(self.operation_list[0])
                self.operation = self.operation_list[0]
                print self.operation_list
                print self.operation
                if self.operation == "RUNNING":
                    if self.ur_state.robot_mode_resp.robot_mode.mode != 7:
                        self.ur.set_robot_to_mode(RobotMode.POWER_OFF)
                        rospy.sleep(0.5)
                        self.ur.set_robot_to_mode(RobotMode.RUNNING)
                        rospy.sleep(10)

                        rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
                        resp = self.ur.s_playProgram()
                        rospy.sleep(0.5)

                    self.operation == None

                elif self.operation == "POWER_OFF":
                    self.ur.set_robot_to_mode(RobotMode.POWER_OFF)
                    rospy.sleep(0.5)

                    rospy.wait_for_service('/ur_hardware_interface/dashboard/stop')
                    resp = self.ur.s_stopProgram()
                    rospy.sleep(0.5)

                    self.operation == None

                elif self.operation == "MOVE_CARTESIAN":
                    print "OPERATION IS MOVE_CARTESIAN"
                    print len(self.pose_list)
                    #print self.pose_list

                    if len(self.pose_list) != 0:
                        pose = self.pose_list[0]
                        print pose
                        self.ur.move_cartesian(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],pose[6])

                        self.cam.save_image('/home/hri/catkin_ws/src/ur_gui/images/', self.num)
                        self.num = self.num + 1

                        del self.pose_list[0]
                        if len(self.pose_list) == 0:
                            #self.pose_list = None
                            self.operation = None
                            self.num = 0

                if self.operation != "MOVE_CARTESIAN":
                    del self.operation_list[0]

                if len(self.operation_list) == 0:
                    self.isRun = False
                    print "Operations are Done!"








