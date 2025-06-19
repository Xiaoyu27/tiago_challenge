#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "std_msgs/msg/string.hpp"

#include "Motionplanning_arms.hpp"
#include "RobotTaskStatus.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <play_motion2_msgs/action/play_motion2.hpp>
#include <control_msgs/action/point_head.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <yaml-cpp/yaml.h>
#include <memory>
#include <chrono>
#include <vector>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create a shared pointer to your node
  auto node = std::make_shared<icr_Motionplanning_arms>();

  node->GripperControl("OPEN");
  // You can spin in a separate thread or just call your function directly
  try
  {
    double lift_value = 0.0; // Example lift value within limits
    // node->TorsoControl(lift_value);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node->get_logger(), "Exception in TorsoControl: %s ", e.what());
  }

  RobotTaskStatus::Status robot_state = RobotTaskStatus::Status::INIT;
  bool running = true;
  int brick_counter = 0;

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
  planning_scene_publisher_ = node->create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", rclcpp::QoS(1));

  // Add obj table in pose 0.4 0.0 0.5
  geometry_msgs::msg::PoseStamped obstacle_pose;
  obstacle_pose.header.frame_id = "base_footprint";
  obstacle_pose.pose.position.x = 0.4;
  obstacle_pose.pose.position.y = 0.0;
  obstacle_pose.pose.position.z = 0.5;
  obstacle_pose.pose.orientation.x = 0.0;
  obstacle_pose.pose.orientation.y = 0.0;
  obstacle_pose.pose.orientation.z = 0.0;
  obstacle_pose.pose.orientation.w = 1.0;
  moveit_msgs::msg::PlanningScene planning_scene_msg =
      node->Add_Obstacle(obstacle_pose, "Table");
  // planning_scene_publisher_->publish(planning_scene_msg);

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.7;

  // Simple orientation (quaternion), facing forward
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 0.7071068;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.7071068;

  // Home pose
  // At time 1750336742.396280013
  // - Translation: [-0.816, 0.688, -0.301]
  // - Rotation: in Quaternion [0.775, -0.002, 0.478, 0.412]
  // - Rotation: in RPY (radian) [1.878, -0.838, 0.626]
  // - Rotation: in RPY (degree) [107.587, -47.995, 35.871]
  // - Matrix:
  //   0.542 -0.397 0.741 -0.816
  //   0.392 -0.660 -0.641  0.688
  //   0.743  0.638 -0.202 -0.301
  //   0.000  0.000 0.000  1.000

  try
  {
    node->motion_planning_control(target_pose, RobotTaskStatus::Arm::ARM_torso); // or "arm" depending on your group name
    RCLCPP_INFO(node->get_logger(), "Motion planning succeeded.");
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
  }

  while (running && rclcpp::ok())
  {
    switch (robot_state)
    {
    case RobotTaskStatus::Status::INIT:
    {
      // move_to_waiting_brick();

      robot_state = RobotTaskStatus::Status::WAITING_BRICK;
      std::cout << "robot_state: " << (int)robot_state << std::endl;
      break;
    }
    case RobotTaskStatus::Status::WAITING_BRICK:
    {
      std::cout << "press enter to grasp" << std::endl;
      std::cin.get();
      node->GripperControl("CLOSE");
      robot_state = RobotTaskStatus::Status::CLOSE_GRIPPER;
      std::cout << "robot_state: " << (int)robot_state << std::endl; // 1
      break;
    }
    case RobotTaskStatus::Status::CLOSE_GRIPPER:
    {
      std::cin.get();
      // move_to_placing_pose();
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = 0.4;
      target_pose.position.y = 0.0;
      target_pose.position.z = 0.7 - 0.03;

      // Simple orientation (quaternion), facing forward
      target_pose.orientation.x = 0.0;
      target_pose.orientation.y = 0.7071068;
      target_pose.orientation.z = 0.0;
      target_pose.orientation.w = 0.7071068;
      node->motion_planning_control(target_pose, RobotTaskStatus::Arm::ARM_torso); // or "arm" depending on your group name
      
      std::cout << "gripper closed, move to place" << std::endl;

      robot_state = RobotTaskStatus::Status::MOVE_TO_PLACE;
      std::cout << "robot_state: " << (int)robot_state << std::endl; // 2
      break;
    };
    case RobotTaskStatus::Status::MOVE_TO_PLACE:
    {
      std::cin.get();

      node->GripperControl("OPEN");
      std::cout << "gripper opened" << std::endl;

      robot_state = RobotTaskStatus::Status::OPEN_GRIPPER;
      std::cout << "robot_state: " << (int)robot_state << std::endl; // 3
      break;
    }
    case RobotTaskStatus::Status::OPEN_GRIPPER:
    {
      std::cin.get();

      // move_to_waiting_brick();
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = 0.4;
      target_pose.position.y = 0.0;
      target_pose.position.z = 0.7 + 0.03;

      // Simple orientation (quaternion), facing forward
      target_pose.orientation.x = 0.0;
      target_pose.orientation.y = 0.7071068;
      target_pose.orientation.z = 0.0;
      target_pose.orientation.w = 0.7071068;
      node->motion_planning_control(target_pose, RobotTaskStatus::Arm::ARM_torso); // or "arm" depending on your group name
      std::cout << "move to waiting pose" << std::endl;

      robot_state = RobotTaskStatus::Status::MOVE_TO_WAITING;
      std::cout << "robot_state: " << (int)robot_state << std::endl; // 4
      break;
    };
    case RobotTaskStatus::Status::MOVE_TO_WAITING:
    {
      std::cin.get();

      node->GripperControl("OPEN");
      std::cout << "I am waiting" << std::endl;

      robot_state = RobotTaskStatus::Status::DONE;
      std::cout << "robot_state: " << (int)robot_state << std::endl; // 5
      break;
    }
    case RobotTaskStatus::Status::DONE:
    {
      std::cin.get();
      brick_counter++;
      robot_state = RobotTaskStatus::Status::WAITING_BRICK;
      std::cout << "robot_state: " << (int)robot_state << std::endl; // 6
      break;
    }
    default:
      running = false;
      break;
      // std::cout << "robot_state: " << robot_state << std::endl;
    }
  }

  node->GripperControl("CLOSE");

  rclcpp::shutdown();
  return 0;
}
