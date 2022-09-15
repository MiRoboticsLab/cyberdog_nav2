cmake_minimum_required(VERSION 3.8)
project(map_label_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(protocol REQUIRED)
find_package(filesystem REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(nav2_map_server REQUIRED)

set(executable_name label_server)
set(library_name ${executable_name}_core)
set(dependencies
  rclcpp
  nav_msgs
  protocol
  filesystem
  cyberdog_common
  nav2_map_server
)

include_directories(include)
add_library(${library_name}
  src/labelserver_node.cpp
  src/label_store.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(${executable_name} src/labelserver.cpp)
target_include_directories(${executable_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${executable_name} PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name}
  ${library_name}
)

install(TARGETS ${library_name}
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
