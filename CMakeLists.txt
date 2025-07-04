cmake_minimum_required(VERSION 3.8)
project(tiago_challenge)

# Set C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Declare dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  play_motion2_msgs
  control_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  hardware_interface
  controller_interface
  tf2_geometry_msgs
  geometry_msgs
)

# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(dep ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
)

add_library(motion_planning_lib
  src/RobotTaskStatus.cpp
  src/Motionplanning_arms.cpp
)
target_include_directories(motion_planning_lib PUBLIC include)
ament_target_dependencies(motion_planning_lib ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Add executable
add_executable(tiago_challenge_exe src/main_motion.cpp)
# Add executable
add_executable(jon_exe src/jon_main_motion.cpp)

# Add executable
add_executable(fancy_exe src/fancy_motion.cpp)


# Link libraries
ament_target_dependencies(tiago_challenge_exe ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(tiago_challenge_exe motion_planning_lib)
# Install target
install(TARGETS tiago_challenge_exe
  DESTINATION lib/${PROJECT_NAME}
)

# Link libraries
ament_target_dependencies(jon_exe ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(jon_exe motion_planning_lib)
# Install target
install(TARGETS jon_exe
  DESTINATION lib/${PROJECT_NAME}
)

# Link libraries
ament_target_dependencies(fancy_exe ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(fancy_exe motion_planning_lib)
# Install target
install(TARGETS fancy_exe
  DESTINATION lib/${PROJECT_NAME}
)


# Export package
ament_package()
