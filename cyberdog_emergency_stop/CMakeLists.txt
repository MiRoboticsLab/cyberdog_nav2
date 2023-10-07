cmake_minimum_required(VERSION 3.8)
project(cyberdog_emergency_stop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(protocol REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED)
# find_package(pcl_msgs REQUIRED)

include_directories(include)

set(dependencies
  sensor_msgs
  std_msgs
  rclcpp
  cyberdog_common
  nav2_msgs
  protocol)
#  pcl_conversions
#  PCL
#  pcl_msgs)

set(sources
  src/main.cpp
  src/emergency_stop.cpp)

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

add_executable(cyberdog_emergency_stop ${sources})
ament_target_dependencies(cyberdog_emergency_stop ${dependencies})
target_link_libraries(cyberdog_emergency_stop ${PCL_LIBRARIES})

install(TARGETS cyberdog_emergency_stop RUNTIME DESTINATION bin)
# install targets
install(TARGETS cyberdog_emergency_stop DESTINATION lib/${PROJECT_NAME})

# install config launch
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
