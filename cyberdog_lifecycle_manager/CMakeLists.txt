cmake_minimum_required(VERSION 3.8)
project(cyberdog_lifecycle_manager)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(cyberdog_lifecycle REQUIRED)
find_package(cyberdog_lifecycle_msgs REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/lifecycle_manager.cpp
  src/lifecycle_manager_client.cpp
)

set(dependencies
  # geometry_msgs
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  cyberdog_lifecycle
  cyberdog_lifecycle_msgs
  bondcpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(cyberdog_lifecycle_manager_node
  src/main.cpp
)

target_link_libraries(cyberdog_lifecycle_manager_node
  ${library_name}
)

ament_target_dependencies(cyberdog_lifecycle_manager_node
  ${dependencies}
)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  cyberdog_lifecycle_manager_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

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

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
