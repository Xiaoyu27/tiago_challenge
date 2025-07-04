#pragma once

#include <string>
#include <geometry_msgs/msg/pose.hpp>

class RobotTaskStatus
{
public:
  // Task status
  enum class Status
  {
    INIT = 0,
    WAITING_BRICK,
    CLOSE_GRIPPER,
    MOVE_TO_PLACE,
    OPEN_GRIPPER,
    MOVE_TO_WAITING,
    DONE
  };

  // Target arm
  enum class Arm
  {
    ARM,
    ARM_torso
  };

  // Constructors
  RobotTaskStatus();
  RobotTaskStatus(const geometry_msgs::msg::Pose & goal, Status status, Arm arm);

  // Setters
  void setGoal(const geometry_msgs::msg::Pose & goal);
  void setStatus(Status status);
  void setArm(Arm arm);

  // Getters
  geometry_msgs::msg::Pose getGoal() const;
  Status getStatus() const;
  Arm getArm() const;

  // String representation helpers
  static std::string statusToString(Status status);
  static std::string armToString(Arm arm);

private:
  geometry_msgs::msg::Pose goal_;
  Status status_;
  Arm arm_;
};
