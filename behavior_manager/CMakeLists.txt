cmake_minimum_required(VERSION 3.8)
project(behavior_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(protocol REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(cyberdog_debug REQUIRED)
find_package(cyberdog_system REQUIRED)
find_package(mcr_msgs REQUIRED)
find_package(motion_action REQUIRED)
find_package(motion_manager REQUIRED)
# find_package(visualization REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_compile_options(-g)
set(dependencies
  rclcpp
  geometry_msgs
  cyberdog_common
  nav2_util
  nav2_lifecycle_manager
  nav2_msgs
  nav_msgs
  nav2_map_server
  rclcpp_lifecycle
  std_msgs
  builtin_interfaces
  action_msgs
  protocol
  rclcpp_action
  rclcpp_components
  mcr_msgs
  motion_action
  cyberdog_system
  # visualization
)

set(sources
  src/main.cpp
)

add_executable(${PROJECT_NAME} ${sources})
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

add_executable(fake_nodes
  test/main.cpp
)
ament_target_dependencies(fake_nodes
  ${dependencies}
)

add_executable(auto_tracking
  test/client_test.cpp
)
ament_target_dependencies(auto_tracking
  ${dependencies}
)

add_executable(targetstatic_test
  test/targetstatic_test.cpp
)
ament_target_dependencies(targetstatic_test
  ${dependencies}
)
# target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

# add_executable(core_client test/motioncore_client.cpp)
# ament_target_dependencies(${PROJECT_NAME}
#   ${dependencies}
# )
# target_link_libraries(core_client ${PROJECT_NAME}_lib)

# set(library_name ${PROJECT_NAME}_lib)
# add_library(${library_name} SHARED
#   ${sources}
# )
# ament_target_dependencies(${library_name}
#   ${dependencies}
# )

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(DIRECTORY include/
  DESTINATION include/)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  fake_nodes
  auto_tracking
  targetstatic_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
endif()

ament_export_include_directories(include)
ament_package()
