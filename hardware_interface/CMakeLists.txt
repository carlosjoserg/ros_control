cmake_minimum_required(VERSION 3.5)
project(hardware_interface)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  include_directories(${rclcpp_INCLUDE_DIRS} include)

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(hardware_resource_manager_test  test/hardware_resource_manager_test.cpp)
  target_link_libraries(hardware_resource_manager_test ${rclcpp_LIBRARIES})

  ament_add_gtest(actuator_state_interface_test   test/actuator_state_interface_test.cpp)
  target_link_libraries(actuator_state_interface_test ${rclcpp_LIBRARIES})

  ament_add_gtest(actuator_command_interface_test test/actuator_command_interface_test.cpp)
  target_link_libraries(actuator_command_interface_test ${rclcpp_LIBRARIES})

  ament_add_gtest(joint_state_interface_test      test/joint_state_interface_test.cpp)
  target_link_libraries(joint_state_interface_test ${rclcpp_LIBRARIES})

  ament_add_gtest(joint_command_interface_test    test/joint_command_interface_test.cpp)
  target_link_libraries(joint_command_interface_test ${rclcpp_LIBRARIES})

  ament_add_gtest(force_torque_sensor_interface_test test/force_torque_sensor_interface_test.cpp)
  target_link_libraries(force_torque_sensor_interface_test ${rclcpp_LIBRARIES})

  ament_add_gtest(imu_sensor_interface_test       test/imu_sensor_interface_test.cpp)
  target_link_libraries(imu_sensor_interface_test ${rclcpp_LIBRARIES})

  ament_add_gtest(robot_hw_test                   test/robot_hw_test.cpp)
  target_link_libraries(robot_hw_test ${rclcpp_LIBRARIES})

  ament_add_gtest(interface_manager_test          test/interface_manager_test.cpp)
  target_link_libraries(interface_manager_test ${rclcpp_LIBRARIES})

  ament_add_gtest(posvel_command_interface_test test/posvel_command_interface_test.cpp)
  target_link_libraries(posvel_command_interface_test ${rclcpp_LIBRARIES})

  ament_add_gtest(posvelacc_command_interface_test test/posvelacc_command_interface_test.cpp)
  target_link_libraries(posvelacc_command_interface_test ${rclcpp_LIBRARIES})

endif()

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)

ament_export_include_directories(include)

ament_package()
