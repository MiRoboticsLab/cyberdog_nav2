cmake_minimum_required(VERSION 3.8)
project(cyberdog_maps_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(protocol REQUIRED)

set(DEPENDENCIES
  rclcpp
  std_msgs
  cyberdog_common
  nav2_util
  nav2_lifecycle_manager
  nav2_msgs
  nav_msgs
  protocol
)

set(SRCS
  src/maps_protocol.cpp
  src/maps_manager.cpp
)

# project
add_executable(${PROJECT_NAME} src/main.cpp ${SRCS})
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# test
add_executable(map_manager_test  test/map_manager_test.cpp ${SRCS})
ament_target_dependencies(map_manager_test ${DEPENDENCIES})

target_include_directories(map_manager_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(map_manager_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME} map_manager_test
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

ament_package()
