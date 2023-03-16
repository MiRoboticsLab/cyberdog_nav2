cmake_minimum_required(VERSION 3.8)
project(nav2_control_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(visualization REQUIRED)
find_package(cyberdog_visions_interfaces REQUIRED)

# dependencies
set(dependencies
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  nav2_msgs
  nav2_util
  visualization
  cyberdog_visions_interfaces)

include_directories(include)

# nav2_mapping
add_executable(nav2_mapping src/nav2_mapping.cpp)
ament_target_dependencies(nav2_mapping ${dependencies})

# nav2_localization
add_executable(nav2_localization src/nav2_localization.cpp)
ament_target_dependencies(nav2_localization ${dependencies})

# nav2_navigatte_pose
add_executable(nav2_navigatte_pose src/nav2_navigatte_pose.cpp)
ament_target_dependencies(nav2_navigatte_pose ${dependencies})

# install
install(TARGETS
  nav2_mapping
  nav2_localization
  nav2_navigatte_pose
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
