cmake_minimum_required(VERSION 2.8.3)

project(conveyor_belt_ros)

# Add support for c++11
add_definitions(-std=c++11)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  lwr_ros_client
  dynamic_reconfigure
  roscpp
  rospy
  serial
  std_msgs
)

# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(cfg/conveyorBelt_params.cfg)


# find_package(Eigen REQUIRED)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

 catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS lwr_ros_client roscpp rospy serial std_msgs dynamic_reconfigure
    # DEPENDS
 )

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(conveyorBelt src/conveyorBelt_node.cpp src/ConveyorBeltController.cpp)
target_link_libraries(conveyorBelt ${catkin_LIBRARIES})
add_dependencies(conveyorBelt ${PROJECT_NAME}_gencfg)
