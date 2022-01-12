cmake_minimum_required(VERSION 3.5)
project(gazebo_conveyor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Protobuf REQUIRED)


link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConveyorBeltControl.srv"
  "msg/ConveyorBeltState.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${Boost_INCLUDE_DIRS})
include_directories(include ${gazebo_dev_INCLUDE_DIRS})
include_directories(${PROTOBUF_INCLUDE_DIR})



set(conveyor_belt_plugin_name ConveyorBeltPlugin)
add_library(${conveyor_belt_plugin_name} src/ConveyorBeltPlugin.cc)
rosidl_target_interfaces(${conveyor_belt_plugin_name}
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${conveyor_belt_plugin_name}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS ${conveyor_belt_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(ros_conveyor_belt_plugin_name ROSConveyorBeltPlugin)
add_library(${ros_conveyor_belt_plugin_name} src/ROSConveyorBeltPlugin.cc)
rosidl_target_interfaces(${ros_conveyor_belt_plugin_name}
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${ros_conveyor_belt_plugin_name}
  ${conveyor_belt_plugin_name}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ${ros_conveyor_belt_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_ros_pkgs)

ament_package()
