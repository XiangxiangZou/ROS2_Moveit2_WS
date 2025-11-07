# include <rclcpp/rclcpp.hpp>
# include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); // Initialize ROS 2 context

  // Create a ROS 2 node
  auto node = std::make_shared<rclcpp::Node>("robot_moveit"); 

  // Create a single-threaded executor and add the node to it
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Spin the executor in a separate thread
  auto spinner = std::thread([&executor]() {
    executor.spin();
  });

  // Create a MoveGroupInterface for the "arm" group
  auto arm_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
  arm_group.setMaxVelocityScalingFactor(1.0);
  arm_group.setMaxAccelerationScalingFactor(1.0);

  // Create a MoveGroupInterface for the "gripper" group
  auto gripper_group = moveit::planning_interface::MoveGroupInterface(node, "gripper");
  gripper_group.setMaxVelocityScalingFactor(1.0);
  gripper_group.setMaxAccelerationScalingFactor(1.0);

  // ------------------------ Named Targets ------------------------
  // // Define start and goal states, plan and execute the motion
  // arm_group.setStartStateToCurrentState(); //setting the start state
  // arm_group.setNamedTarget("pose_1"); //set the goal state to "pose_1"
  // //plan the path from start state to goal state
  // moveit::planning_interface::MoveGroupInterface::Plan plan1; 
  // bool success1 = (arm_group.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS) ;
  // //if the plan is successful, execute the plan
  // if (success1)
  // {
  //   arm_group.execute(plan1);
  // }


  // arm_group.setStartStateToCurrentState(); 
  // arm_group.setNamedTarget("home"); 


  // arm_group.setStartStateToCurrentState(); 
  // arm_group.setNamedTarget("pose_2"); 
  // moveit::planning_interface::MoveGroupInterface::Plan plan2; 
  // bool success2 = (arm_group.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS) ;
  // if (success2)
  // {
  //   arm_group.execute(plan2);
  // }
  // --------------------------------------------------------------------------

  // ------------------------ Joint Targets ------------------------
  // std::vector<double> joint_group_positions = {1.5,0.5,0.0,1.5,0.0,-0.75};
  // arm_group.setStartStateToCurrentState();
  // arm_group.setJointValueTarget(joint_group_positions);
  // moveit::planning_interface::MoveGroupInterface::Plan plan1;
  // bool success1 = (arm_group.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
  // if (success1)
  // {
  //   arm_group.execute(plan1);
  // }
  // --------------------------------------------------------------------------

  // ------------------------ Pose Targets ------------------------

  // Define a quaternion
  tf2::Quaternion q;
  q.setRPY(3.14, 0.0, 0.0);
  q = q.normalize(); // Normalize the quaternion

  // Define the target pose
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.pose.position.x = 0.7;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = 0.4;
  target_pose.pose.orientation.x = q.getX();
  target_pose.pose.orientation.y = q.getY();
  target_pose.pose.orientation.z = q.getZ();
  target_pose.pose.orientation.w = q.getW();
  
  arm_group.setStartStateToCurrentState();
  arm_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  bool success = (arm_group.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success)
  {
    arm_group.execute(plan1);
  }

  // --------------------------------------------------------------------------

  // ------------------------ Cartesian Path ------------------------
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose pose1 = arm_group.getCurrentPose().pose;
  pose1.position.z += -0.2; // First waypoint: down
  waypoints.push_back(pose1);

  geometry_msgs::msg::Pose pose2 = pose1;
  pose2.position.y += 0.2; // Second waypoint: right
  waypoints.push_back(pose2);

  geometry_msgs::msg::Pose pose3 = pose2;
  pose3.position.y += -0.2; // Third waypoint: left
  pose3.position.z += 0.2; // and up
  waypoints.push_back(pose3);

  moveit_msgs::msg::RobotTrajectory trajectory;

  double fraction = arm_group.computeCartesianPath(
    waypoints,
    0.01,  // eef_step
    trajectory);

  if (fraction == 1.0)
  {
    arm_group.execute(trajectory);
    RCLCPP_INFO(node->get_logger(), "Path computed successfully. Moving the arm.");
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Only able to compute %.0f%% of the path. Not executing.", fraction * 100.0);
  }
  // --------------------------------------------------------------------------






  rclcpp::shutdown(); // Shutdown ROS 2 context
  spinner.join(); // Wait for the spinner thread to finish
  return 0;
}




