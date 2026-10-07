cmake_minimum_required(VERSION 3.22)
project(lbr_demos_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_msgs REQUIRED)
find_package(FRIClient REQUIRED)
find_package(lbr_fri_idl REQUIRED)
find_package(lbr_fri_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

# joint sine overlay
add_executable(joint_sine_overlay
  src/joint_sine_overlay.cpp
)

ament_target_dependencies(joint_sine_overlay
  lbr_fri_idl
  lbr_fri_ros2
  rclcpp
)

target_link_libraries(joint_sine_overlay
  FRIClient::FRIClient
)

# joint trajectory client
add_executable(joint_trajectory_client
  src/joint_trajectory_client.cpp
)

ament_target_dependencies(joint_trajectory_client
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

target_link_libraries(joint_trajectory_client
  FRIClient::FRIClient
)

# torque sine overlay
add_executable(torque_sine_overlay
  src/torque_sine_overlay.cpp
)

ament_target_dependencies(torque_sine_overlay
  lbr_fri_idl
  lbr_fri_ros2
  rclcpp
)

target_link_libraries(torque_sine_overlay
  FRIClient::FRIClient
)

# wrench sine overlay
add_executable(wrench_sine_overlay
  src/wrench_sine_overlay.cpp
)

ament_target_dependencies(wrench_sine_overlay
  lbr_fri_idl
  lbr_fri_ros2
  rclcpp
)

target_link_libraries(wrench_sine_overlay
  FRIClient::FRIClient
)

# Standalone direct FRI POSITION client matching Sunrise's
# FRIJointPositionImpedance application. Robot-reference shaping and target
# freshness checks run in the native FRI callback.
add_executable(kuka_fri_joint_impedance
  ../../../../control/joint_impedance/src/kuka_fri_joint_impedance.cpp
)
target_include_directories(kuka_fri_joint_impedance PRIVATE
  ../../../../control/joint_impedance/include
)
target_link_libraries(kuka_fri_joint_impedance PRIVATE
  FRIClient::FRIClient
  yaml-cpp
)

# Opt-in FRI torque client. The cabinet must run the matching Sunrise torque-overlay app.
add_executable(fri_cartesian_wall_client
  src/fri_cartesian_wall_client.cpp
)

ament_target_dependencies(fri_cartesian_wall_client
  lbr_fri_ros2
  rclcpp
  sensor_msgs
  std_srvs
)

target_link_libraries(fri_cartesian_wall_client
  Eigen3::Eigen
  FRIClient::FRIClient
)

install(TARGETS
  joint_sine_overlay
  joint_trajectory_client
  torque_sine_overlay
  wrench_sine_overlay
  kuka_fri_joint_impedance
  fri_cartesian_wall_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY sunrise/ DESTINATION share/${PROJECT_NAME}/sunrise)
install(FILES CARTESIAN_WALL.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cartesian_wall_controller
    test/test_cartesian_wall_controller.cpp
  )
  target_link_libraries(test_cartesian_wall_controller Eigen3::Eigen)
  target_include_directories(test_cartesian_wall_controller PRIVATE include)
endif()

ament_package()
