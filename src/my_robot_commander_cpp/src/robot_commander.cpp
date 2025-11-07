# include <rclcpp/rclcpp.hpp>
# include <moveit/move_group_interface/move_group_interface.hpp>
# include <example_interfaces/msg/bool.hpp>
# include <example_interfaces/msg/float64_multi_array.hpp>
# include <my_robot_interfaces/msg/pose_cmd.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using Float64MultiArray = example_interfaces::msg::Float64MultiArray; 
using PoseCmd = my_robot_interfaces::msg::PoseCmd;
using namespace std::placeholders;


class RobotCommander
{
public:
  RobotCommander(std::shared_ptr<rclcpp::Node> node)
  {
    node_ = node;
    arm_group_ = std::make_shared<MoveGroupInterface>(node_, "arm");
    arm_group_->setMaxVelocityScalingFactor(1.0);
    arm_group_->setMaxAccelerationScalingFactor(1.0);

    gripper_group_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

    open_gripper_sub_ = node_->create_subscription<Bool>(
      "open_gripper", 10, std::bind(&RobotCommander::OpenGripperCallback, this, _1));

    joint_cmd_sub_ = node_->create_subscription<Float64MultiArray>(
      "joint_cmd", 10, std::bind(&RobotCommander::JointCmdCallback, this, _1));

    pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
      "pose_cmd", 10, std::bind(&RobotCommander::PoseCmdCallback, this, _1));


  }

  void goToNamedTarget(const std::string &target_name)
  {
    arm_group_->setStartStateToCurrentState();
    arm_group_->setNamedTarget(target_name);
    planAndExecute(arm_group_);
  }

  void goToJointTarget(const std::vector<double> &joint_group_positions)
  {
    arm_group_->setStartStateToCurrentState();
    arm_group_->setJointValueTarget(joint_group_positions);
    planAndExecute(arm_group_);
  }

  void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path = false){
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm_group_->setStartStateToCurrentState();

    if (!cartesian_path){
      arm_group_->setPoseTarget(target_pose);
      planAndExecute(arm_group_);
    }
    else{
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target_pose.pose);
      moveit_msgs::msg::RobotTrajectory trajectory;
      double fraction = arm_group_->computeCartesianPath(waypoints, 0.01, trajectory);
      RCLCPP_INFO(node_->get_logger(), "Cartesian path (%.2f%% achieved)", fraction * 100.0);
      if (fraction == 1.0)
      {
        arm_group_->execute(trajectory);
        RCLCPP_INFO(node_->get_logger(), "Path computed successfully. Moving the arm.");
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "Only able to compute %.0f%% of the path. Not executing.", fraction * 100.0);
      }
    }
  }

  void openGripper()
  {
    gripper_group_->setStartStateToCurrentState();
    gripper_group_->setNamedTarget("gripper_open");
    planAndExecute(gripper_group_);
  }
  void closeGripper()
  {
    gripper_group_->setStartStateToCurrentState();
    gripper_group_->setNamedTarget("gripper_close");
    planAndExecute(gripper_group_);
  }

  
private:

  void planAndExecute(const std::shared_ptr<MoveGroupInterface> &move_group)
  {
    MoveGroupInterface::Plan plan;
    bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
      move_group->execute(plan);
    }
  }

  void OpenGripperCallback(const Bool &msg)
  {
    if (msg.data) {
      openGripper();
    } else {
      closeGripper();
    }
  }

  void JointCmdCallback(const Float64MultiArray &msg)
  {
    auto joint_target = msg.data;
    
    if (joint_target.size() != arm_group_->getJointNames().size()) {
      RCLCPP_ERROR(node_->get_logger(), "Joint command size does not match arm group size.");
      return;
    }
    goToJointTarget(joint_target);
  }

  void PoseCmdCallback(const PoseCmd &msg)
  {
    goToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MoveGroupInterface> arm_group_;
  std::shared_ptr<MoveGroupInterface> gripper_group_;

  rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
  rclcpp::Subscription<Float64MultiArray>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
  
}; 






int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); // Initialize ROS 2 context
  auto node = std::make_shared<rclcpp::Node>("robot_commander");
  auto robot_commander = RobotCommander(node);
  rclcpp::spin(node); // Spin the node to process callbacks
  rclcpp::shutdown(); // Shutdown ROS 2 context
  return 0;

}




