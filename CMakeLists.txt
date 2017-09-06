cmake_minimum_required(VERSION 2.8.3)
project(move_base_to_manip)


add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  std_srvs
  tf
  tf2_geometry_msgs
)

add_action_files(
  DIRECTORY action
  FILES desired_pose.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    move_base_msgs
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    roscpp
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(provide_target src/provide_target.cpp)
add_dependencies(provide_target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(provide_target ${catkin_LIBRARIES} )

add_executable(move_base_to_manip src/move_base_to_manip.cpp)
add_dependencies(move_base_to_manip ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(move_base_to_manip ${catkin_LIBRARIES} )


#############
## Install ##
#############

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

install(TARGETS move_base_to_manip provide_target
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY action
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


