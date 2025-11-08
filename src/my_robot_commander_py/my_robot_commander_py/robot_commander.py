import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped
import tf_transformations

from example_interfaces.msg import Bool
from example_interfaces.msg import Float64MultiArray
from my_robot_interfaces.msg import PoseCmd

ROBOT_CONFIG = MoveItConfigsBuilder(robot_name="my_robot", package_name="my_robot_moveit_config")\
                                    .robot_description_semantic("config/my_robot.srdf", {"name": "my_robot"})\
                                    .to_dict()

ROBOT_CONFIG = { 
    **ROBOT_CONFIG,
    "planning_scene_monitor": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
        "planning_pipelines": {
            "pipeline_names": ["ompl"]
        },
        "plan_request_params": {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 1.0,
            "max_acceleration_scaling_factor": 1.0
        },
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": ["default_planning_request_adapters/ResolveConstraintFrames",
                            "default_planning_request_adapters/ValidateWorkspaceBounds",
                            "default_planning_request_adapters/CheckStartStateBounds",
                            "default_planning_request_adapters/CheckStartStateCollision"],
            "response_adapters": ["default_planning_response_adapters/AddTimeOptimalParameterization",
                             "default_planning_response_adapters/ValidateSolution",
                             "default_planning_response_adapters/DisplayMotionPath"],
            "start_state_max_bounds_error": 0.1
        }
}

class RobotCommander(Node):
    def __init__(self):
        super().__init__("robot_commander_node")
        self.robot_ = MoveItPy(
            node_name="robot_commander_py",
            config_dict=ROBOT_CONFIG
        )
        self.arm_ : PlanningComponent = self.robot_.get_planning_component("arm")
        self.gripper_ : PlanningComponent = self.robot_.get_planning_component("gripper")

        self.open_gripper_sub = self.create_subscription(
            msg_type=Bool,
            topic="open_gripper_py",
            callback=self.open_gripper_callback,
            qos_profile=10
        )

        self.joint_cmd_sub = self.create_subscription(
            msg_type=Float64MultiArray,
            topic="joint_cmd_py",
            callback=self.joint_cmd_callback,
            qos_profile=10
        )

        self.pose_cmd_sub = self.create_subscription(
            msg_type=PoseCmd,
            topic="pose_cmd_py",
            callback=self.pose_cmd_callback,
            qos_profile=10
        )

    def go_to_named_target(self, target_name):
        self.arm_.set_start_state_to_current_state()
        self.arm_.set_goal_state(configuration_name=target_name)
        self.plan_and_execute(self.arm_)

    def go_to_joint_target(self, joint_positions):   
        if len(joint_positions) != 6:
            self.get_logger().error("go_to_joint_target: joint_positions must be 6")
            return
        robot_state = RobotState(self.robot_.get_robot_model())     
        joint_values = {
            "joint1": joint_positions[0],
            "joint2": joint_positions[1],
            "joint3": joint_positions[2],
            "joint4": joint_positions[3],
            "joint5": joint_positions[4],
            "joint6": joint_positions[5],
        }
        robot_state.joint_positions = joint_values
        self.arm_.set_start_state_to_current_state()
        self.arm_.set_goal_state(robot_state=robot_state)
        self.plan_and_execute(self.arm_)

    def go_to_pose_target(self, x, y, z, roll, pitch, yaw):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        self.arm_.set_start_state_to_current_state()
        self.arm_.set_goal_state(pose_stamped_msg=pose, pose_link="tool_link")
        self.plan_and_execute(self.arm_)

    def open_gripper(self):
        self.gripper_.set_start_state_to_current_state()
        self.gripper_.set_goal_state(configuration_name="gripper_open")
        self.plan_and_execute(self.gripper_)

    def close_gripper(self):
        self.gripper_.set_start_state_to_current_state()
        self.gripper_.set_goal_state(configuration_name="gripper_close")
        self.plan_and_execute(self.gripper_)
        
    def plan_and_execute(self, interface):
        plan_result = interface.plan()
        if plan_result:
            self.get_logger().info("Plan success, and now executing trajectory")
            self.robot_.execute(plan_result.trajectory,controllers=[])
        else:
            self.get_logger().error("Plan failed, check the target whether it is in the workspace or the target is not colliding with the obstacles")

    def open_gripper_callback(self, msg: Bool):
        if msg.data:
            self.open_gripper()
        else:
            self.close_gripper()

    def joint_cmd_callback(self, msg: Float64MultiArray):
        if msg.data.__len__() != 6:
            self.get_logger().error(f"joint_cmd_callback: joint_cmd must be 6")
            return
        self.go_to_joint_target(msg.data)

    def pose_cmd_callback(self, msg: PoseCmd):
        self.go_to_pose_target(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw)




def main(args=None):
    rclpy.init(args=args)
    node = RobotCommander()
    rclpy.spin(node)
    rclpy.shutdown()