cmake_minimum_required(VERSION 3.22)
project(lbr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/admittance_telemetry_bridge
    scripts/commanded_pose_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

# Launch mixins
ament_python_install_package(lbr_bringup)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_telemetry_timing test/test_telemetry_timing.py)
  ament_add_pytest_test(test_telemetry_bridge test/test_telemetry_bridge.py
    ENV ROS_DOMAIN_ID=172 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
endif()

ament_package()
