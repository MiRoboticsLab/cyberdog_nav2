cmake_minimum_required(VERSION 3.5)
project(tracking_base)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()
# add_compile_options(-g -O0)

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(dwb_critics REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
#find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(teb_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mcr_msgs REQUIRED)
find_package(teb_local_planner REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(Boost REQUIRED)

# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EXTERNAL_INCLUDE_DIRS}
)

## Build the tracking_base_core library

add_executable(tracking_base
   src/target_manager.cpp
   src/tracking_base.cpp
   src/main.cpp
)

set(ament_dependencies
    costmap_converter
    dwb_critics
    nav_2d_utils
    nav2_core
    nav2_costmap_2d
    nav2_util
    geometry_msgs
    nav_msgs
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    std_msgs
    pluginlib
    teb_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    mcr_msgs
    teb_local_planner
    visualization_msgs
    builtin_interfaces
)
ament_target_dependencies(tracking_base
   ${ament_dependencies}
)
target_link_libraries(tracking_base
   ${EXTERNAL_LIBS}
)

target_compile_definitions(tracking_base PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
## Build additional executables

# add_executable(test_optim_node src/test_optim_node.cpp)

# target_link_libraries(test_optim_node
#    tracking_base
#    ${EXTERNAL_LIBS}
#    ${ament_dependencies}
# )


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS tracking_base
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# install(TARGETS tracking_base
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION include/
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   tracking_base_core_plugin.xml
#   DESTINATION share
#   )

# install(PROGRAMS scripts/cmd_vel_to_ackermann_drive.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
# ament_export_libraries(tracking_base_core)
ament_export_dependencies(${ament_dependencies})
# pluginlib_export_plugin_description_file(nav2_core tracking_base_core_plugin.xml)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  #ament_add_gtest(homotopy_class_planner_test
  #    test/homotopy_class_planner_test.cpp
  #)
  #target_link_libraries(homotopy_class_planner_test
  #    tracking_base_core
  #)
endif()

ament_package()
