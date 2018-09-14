cmake_minimum_required(VERSION 2.8.3)
project(gripper3f_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotiq_s_model_control
  robotiq_joint_state_publisher
  robotiq_s_model_visualization
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
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gripper3f_control
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(gripper3f_control
	src/gripper3f_control.cpp
)
target_link_libraries(gripper3f_control ${catkin_LIBRARIES})

install(TARGETS gripper3f_control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
   	   PATTERN ".svn" EXCLUDE
   )
        
###########
## Build ##
###########
link_directories(${catkin_LIBRARY_DIRS})

add_executable(gripper_node_sample src/gripper_node_sample.cpp)
target_link_libraries(gripper_node_sample ${catkin_LIBRARIES} gripper3f_control)
