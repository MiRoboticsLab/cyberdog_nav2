cmake_minimum_required(VERSION 3.8)
project(cyberdog_map_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(protocol REQUIRED)
find_package(cyberdog_common REQUIRED)

# dependencies
set(DEPS
  rclcpp
  nav2_util
  std_srvs
  nav_msgs
  protocol
  cyberdog_common
)

# sources
set(SRCS
  src/map_manager.cpp
)

include_directories(include)

add_executable(cyberdog_map_manager src/main.cpp ${SRCS})
ament_target_dependencies(cyberdog_map_manager ${DEPS})

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

install(TARGETS cyberdog_map_manager
  DESTINATION lib/cyberdog_map_manager)

ament_export_include_directories(include)
ament_package()
