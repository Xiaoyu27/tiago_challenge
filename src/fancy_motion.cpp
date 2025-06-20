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
#include <string>
#include <iostream>

// Parameters to set during the demo
double brick_length = 0.05; // in cm
double brick_height = 0.03; // in cm


geometry_msgs::msg::Pose_<std::allocator<void> > get_grasp_pose(){
  geometry_msgs::msg::Pose grasp_pose;
  grasp_pose.position.x = 0.6;
  grasp_pose.position.y = 0.1;
  grasp_pose.position.z = 1.12;

  // Simple orientation (quaternion), facing forward
  grasp_pose.orientation.x = 0.0;
  grasp_pose.orientation.y = 0.0;
  grasp_pose.orientation.z =  0.7071068;
  grasp_pose.orientation.w = 0.7071068;

  return grasp_pose;
}

geometry_msgs::msg::Pose_<std::allocator<void> > get_start_pose(){
  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = 0.6;
  start_pose.position.y = 0.0;
  start_pose.position.z = 1.12;

  // Simple orientation (quaternion), facing forward
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.7071068;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 0.7071068;

  return start_pose;
}

geometry_msgs::msg::Pose_<std::allocator<void> > get_fancy_start_pose(int brick_nr){
  geometry_msgs::msg::Pose start_pose;

  int remainder = brick_nr % 4;

  if (remainder == 0) {
      std::cout << "Pose 1" << std::endl;
      start_pose.position.x = 0.6;
      start_pose.position.y = brick_length/2;
      start_pose.position.z = 1.12;

      // Simple orientation (quaternion), facing forward
      start_pose.orientation.x = -0.5;
      start_pose.orientation.y = 0.5;
      start_pose.orientation.z =  0.5;
      start_pose.orientation.w = 0.5;

    

  } else if (remainder == 1) {
      std::cout << "Pose 2" << std::endl;
      start_pose.position.x = 0.6;
      start_pose.position.y = -brick_length/2;
      start_pose.position.z = 1.12;

      // Simple orientation (quaternion), facing forward
      start_pose.orientation.x = -0.5;
      start_pose.orientation.y = 0.5;
      start_pose.orientation.z =  0.5;
      start_pose.orientation.w = 0.5;

  } else if (remainder == 2) {
      std::cout << "Pose 3" << std::endl;
      start_pose.position.x = 0.6;
      start_pose.position.y = 0.0;
      start_pose.position.z = 1.12;

      // Simple orientation (quaternion), facing forward
      start_pose.orientation.x = 0.0;
      start_pose.orientation.y = 0.7071068;
      start_pose.orientation.z = 0.0;
      start_pose.orientation.w = 0.7071068;

  } else if (remainder == 3) {
      std::cout << "Pose 4" << std::endl;
      start_pose.position.x = 0.6 - brick_length;
      start_pose.position.y = 0.1;
      start_pose.position.z = 1.12;

      // Simple orientation (quaternion), facing forward
      start_pose.orientation.x = 0.0;
      start_pose.orientation.y = 0.7071068;
      start_pose.orientation.z = 0.0;
      start_pose.orientation.w = 0.7071068;

  } else {
      std::cout << "Unexpected case.\n";
      std::cout << "Going to default pose.\n";

      start_pose.position.x = 0.6;
      start_pose.position.y = 0.0;
      start_pose.position.z = 1.12;

      // Simple orientation (quaternion), facing forward
      start_pose.orientation.x = 0.0;
      start_pose.orientation.y = 0.7071068;
      start_pose.orientation.z = 0.0;
      start_pose.orientation.w = 0.7071068;
  }
  return start_pose;
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create a shared pointer to your node
  auto node = std::make_shared<icr_Motionplanning_arms>();

  node->GripperControl("OPEN");
  // You can spin in a separate thread or just call your function directly

  double lift_value = .5; // Example lift value within limits

  // To account for the first addition when 0 % 2 = 0
  double bottom_lift_value = - brick_height;
  try
  {
    node->TorsoControl(lift_value);
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
  obstacle_pose.pose.position.z = .53;
  obstacle_pose.pose.orientation.x = 0.0;
  obstacle_pose.pose.orientation.y = 0.0;
  obstacle_pose.pose.orientation.z = 0.0;
  obstacle_pose.pose.orientation.w = 1.0;
  moveit_msgs::msg::PlanningScene planning_scene_msg =
      node->Add_Obstacle(obstacle_pose, "Table");
  planning_scene_publisher_->publish(planning_scene_msg);
  
  node->GripperControl("OPEN");


  // try
  // {
  //   node->motion_planning_control(start_pose, RobotTaskStatus::Arm::ARM); // or "arm" depending on your group name
  //   RCLCPP_INFO(node->get_logger(), "Motion planning succeeded.");
  // }
  // catch (const std::exception &e)
  // {
  //   RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
  // }

  while (running && rclcpp::ok())
  {
    switch (robot_state)
    {

    // Initial State, where the robot moves to the position to pick up the brick
    case RobotTaskStatus::Status::INIT:
    {
      // move_to_waiting_brick();

      geometry_msgs::msg::Pose grasp_pose = get_grasp_pose();
      node->motion_planning_control(grasp_pose, RobotTaskStatus::Arm::ARM); // or "arm" depending on your group name
      std::cout << "robot_state: " << (int)robot_state << std::endl;

      std::cout << "press enter to grasp" << std::endl;

      std::cin.get();
      node->GripperControl("CLOSE");
      robot_state = RobotTaskStatus::Status::WAITING_BRICK;
      std::cout << "robot_state: " << (int)robot_state << std::endl; // 1
      break;
    }

    // State where the gripper gets closed
    case RobotTaskStatus::Status::WAITING_BRICK:
    {
      std::cout << "gripper closed, press enter move to intermediate pose" << std::endl;
      std::cout << "Press 1 to repeat grasp" << std::endl;

      std::string input;
      std::getline(std::cin, input);

      if (input == "1") {
        std::cout << "Going back to pre-grasp";
        robot_state = RobotTaskStatus::Status::INIT;
        std::cout << "robot_state: " << (int)robot_state << std::endl;

      } else {
        std::cout << "Moving to intermediate pose.\n";
        geometry_msgs::msg::Pose grasp_pose = get_fancy_start_pose(brick_counter);
        node->motion_planning_control(grasp_pose, RobotTaskStatus::Arm::ARM); // or "arm" depending on your group name
        robot_state = RobotTaskStatus::Status::CLOSE_GRIPPER;
      } 
      
      break;
    }

    // State where the gripper is closed
    // - moves the robot to the plave position
    case RobotTaskStatus::Status::CLOSE_GRIPPER:
    {
      std::cout << "press enter to go to place pose" << std::endl;
      std::cin.get();

      // Increase the placing location if its the start of the new layer
      if (brick_counter % 2 == 0) {
        bottom_lift_value += brick_height ;
      }
      std::cout << "Move to height: "<< bottom_lift_value << std::endl;

      node->TorsoControl(bottom_lift_value);

      robot_state = RobotTaskStatus::Status::MOVE_TO_PLACE;
      std::cout << "robot_state: " << (int)robot_state << std::endl;
      
      break;
    };

    // State where at the place position
    // - Here the gripper opens to drop the brick
    
    case RobotTaskStatus::Status::MOVE_TO_PLACE:
    {
     
      std::cout << "press enter to open gripper" << std::endl;
      std::cin.get();
      

      node->GripperControl("OPEN");
      std::cout << "gripper opened" << std::endl;

      robot_state = RobotTaskStatus::Status::OPEN_GRIPPER;
      std::cout << "robot_state: " << (int)robot_state << std::endl; // 3
      break;
    }
    case RobotTaskStatus::Status::OPEN_GRIPPER:
    {

       // Move back up

      std::cout << "Gripper opened, press enter move to start pose" << std::endl;
      std::cout << "Press 1 to repeat opening of grasp" << std::endl;

      std::string input;
      std::getline(std::cin, input);

      if (input == "1") {
        std::cout << "Going back to pre-opening";
        robot_state = RobotTaskStatus::Status::MOVE_TO_PLACE;
        std::cout << "robot_state: " << (int)robot_state << std::endl;

      } else {
        std::cout << "Moving to start pose.\n";
        
        // double lift_value = .5; // Example lift value within limits/
        // Raise the torso back up
        node->TorsoControl(lift_value);

        robot_state = RobotTaskStatus::Status::DONE;
        std::cout << "robot_state: " << (int)robot_state << std::endl; // 4
      } 

      break;
    };
    case RobotTaskStatus::Status::DONE:
    {
      std::cin.get();
      brick_counter++;
      robot_state = RobotTaskStatus::Status::INIT;
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
