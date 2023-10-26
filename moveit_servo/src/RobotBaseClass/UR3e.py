# install necessary library:
import sys, rospy, actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
    "twist_controller",
]

CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class TrajectoryClientUR:
    
    def __init__(self):
        rospy.init_node("test_move")

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[3]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[3]

    
    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)
    
    def send_joint_trajectory(self):
        """Creates a trajectory and sends it using the selected action server"""

        # make sure the correct controller is loaded and activated
        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        # The following list are arbitrary positions
        # Change to your own needs if desired
        position_list = [[0, -1.57, -1.57, 0, 0, 0]]
        position_list.append([0.2, -1.57, -1.57, 0, 0, 0])
        position_list.append([-0.5, -1.57, -1.2, 0, 0, 0])

        velocity_list = [[0.02, 0, 0, 0, 0, 0]]
        velocity_list.append([-0.02, 0, 0, 0, 0, 0])
        velocity_list.append([0, 0, 0, 0, 0, 0])
        duration_list = [3.0, 7.0, 10.0]
        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.velocities = velocity_list[i]
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def send_cartesian_trajectory(self):
        """Creates a Cartesian trajectory and sends it using the selected action server"""
        self.switch_controller(self.cartesian_trajectory_controller)

        # make sure the correct controller is loaded and activated
        goal = FollowCartesianTrajectoryGoal()
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # The following list are arbitrary positions
        # Change to your own needs if desired
        pose_list = [
            geometry_msgs.Pose(
                geometry_msgs.Vector3(0.4, -0.1, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
            geometry_msgs.Pose(
                geometry_msgs.Vector3(0.4, -0.1, 0.6), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
            geometry_msgs.Pose(
                geometry_msgs.Vector3(0.4, 0.3, 0.6), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
            geometry_msgs.Pose(
                geometry_msgs.Vector3(0.4, 0.3, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
            geometry_msgs.Pose(
                geometry_msgs.Vector3(0.4, -0.1, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
        ]
        duration_list = [3.0, 4.0, 5.0, 6.0, 7.0]
        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        rospy.loginfo(
            "Executing trajectory using the {}".format(self.cartesian_trajectory_controller)
        )
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()

        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def send_vel_control(self):
         self.switch_controller(self.cartesian_trajectory_controller)
    


if __name__ == "__main__":
    client = TrajectoryClientUR()
    client.switch_controller(client.cartesian_trajectory_controller)