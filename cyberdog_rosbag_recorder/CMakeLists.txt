cmake_minimum_required(VERSION 3.8)
project(cyberdog_rosbag_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcpputils REQUIRED)
find_package(protocol REQUIRED)
find_package(cyberdog_common REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  std_msgs
  rcpputils
  protocol
  cyberdog_common)

set(sources
  src/main.cpp
  src/rosbag_record.cpp)

# rosbag_record
add_executable(rosbag_recorder ${sources})
ament_target_dependencies(rosbag_recorder ${dependencies})

# install targets
install(TARGETS rosbag_recorder DESTINATION lib/${PROJECT_NAME})

# install config launch
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
