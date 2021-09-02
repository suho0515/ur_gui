#QThread
from PyQt5.QtCore import QThread

# ROS
import rospy
import actionlib
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
import tf2_ros


from std_srvs.srv import Trigger
import std_msgs.msg
from std_msgs.msg import Bool

from scipy.spatial.transform import Rotation

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


class URState(QThread):
    def __init__(self, parent=None):
        super(URState,self).__init__()
        self.main = parent
        self.isRun = False

        print("URState Class is Initializing")

        self.s_getRobotMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)
        self.s_getProgramState = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        self.s_getLoadedProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram)
        self.s_getSafetyMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)

        self.robot_mode_resp = self.s_getRobotMode()
        self.program_state_resp = self.s_getProgramState()
        self.loaded_program_resp = self.s_getLoadedProgram()
        self.safety_mode_resp = self.s_getSafetyMode()



        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)


        self.trans = None
        #self.trans = self.tfBuffer.lookup_transform('base', 'tool0_controller', rospy.Time())

    def run(self):
        while self.isRun:
            self.robot_mode_resp = self.s_getRobotMode()
            self.program_state_resp = self.s_getProgramState()
            self.loaded_program_resp = self.s_getLoadedProgram()
            self.safety_mode_resp = self.s_getSafetyMode()

            #self.trans = self.tfBuffer.lookup_transform('base', 'tool0_controller', rospy.Time())

            rospy.sleep(1000)




class UR():
    def __init__(self):
        print("UR Class is Initializing")

        self.pose_list = []
        self.operation = None
        self.operation_list = []




        # ROS Service Initialization
        self.s_getRobotMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)
        self.s_getProgramState = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        self.s_getLoadedProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram)
        self.s_getSafetyMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)

        self.s_playProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        self.s_stopProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)
        self.s_connectToDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)
        self.s_quitFromDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)
        self.s_loadProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)

        self.s_loadProgram("/programs/ros.urp")

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

        self.switch_controllers_client = rospy.ServiceProxy('/controller_manager/switch_controller',
                SwitchController)
        try:
            self.switch_controllers_client.wait_for_service(timeout)
            print("controller switch service is on")
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach controller switch service. Make sure that the driver is actually running."
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



        self.script_publisher = rospy.Publisher("/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1)





        # Running up the Manipulator
        self.set_robot_to_mode(RobotMode.POWER_OFF)
        rospy.sleep(0.5)
        self.set_robot_to_mode(RobotMode.RUNNING)
        rospy.sleep(10)

        rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
        resp = self.s_playProgram()
        rospy.sleep(0.5)

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

    def move_cartesian(self,x,y,z,rx,ry,rz,time_from_start):
        goal = FollowCartesianTrajectoryGoal()
        point = CartesianTrajectoryPoint()

        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = z

        rot = Rotation.from_euler('xyz', [rx, ry, rz], degrees=True)
        rot_quat = rot.as_quat()
        #print(rot_quat)

        point.pose.orientation.x = rot_quat[0]
        point.pose.orientation.y = rot_quat[1]
        point.pose.orientation.z = rot_quat[2]
        point.pose.orientation.w = rot_quat[3]

        point.time_from_start = rospy.Duration(time_from_start)
        goal.trajectory.points.append(point)
        goal.goal_time_tolerance = rospy.Duration(0.6)

        self.cartesian_trajectory_client.send_goal(goal)
        self.cartesian_trajectory_client.wait_for_result()
        #print(self.cartesian_trajectory_client.get_result())

        #rospy.loginfo("Received result SUCCESSFUL")


    def finalize(self):
        self.set_robot_to_mode(RobotMode.POWER_OFF)
        rospy.sleep(0.5)

