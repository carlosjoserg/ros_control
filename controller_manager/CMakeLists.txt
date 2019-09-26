cmake_minimum_required(VERSION 3.5)
project(controller_manager)

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
find_package(ament_cmake_python REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

message("controller interface headers ${controller_interface_INCLUDE_DIRS}")

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${controller_interface_INCLUDE_DIRS}
  ${controller_manager_msgs_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}

)

add_library(${PROJECT_NAME}
  src/controller_manager.cpp
  include/controller_manager/controller_manager.h
  include/controller_manager/controller_loader_interface.h
  include/controller_manager/controller_loader.h
)

target_link_libraries(${PROJECT_NAME}
  ${pluginlib_LIBRARIES}
  ${Boost_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
#
#  find_package(ament_cmake_gtest REQUIRED)
#  ament_add_gtest(controller_manager_hwi_switch_test
#    test/hwi_switch_test.test
#    test/hwi_switch_test.cpp
#  )
#  target_link_libraries(controller_manager_hwi_switch_test ${PROJECT_NAME} ${catkin_LIBRARIES})

endif()

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(PROGRAMS
  scripts/controller_group
  scripts/controller_manager
  scripts/spawner
  scripts/unspawner
  DESTINATION bin)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(controller_interface)
ament_export_dependencies(controller_manager_msgs)
ament_export_dependencies(hardware_interface)
ament_export_dependencies(pluginlib)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
