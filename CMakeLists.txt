cmake_minimum_required(VERSION 2.8.3)
project(advanced_robotics_franka_controllers)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../rbdl" )
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )


INCLUDE(cmake/FindRBDL.cmake)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  roslib
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

###########
## Build ##
###########

include_directories(
  include
    ${RBDL_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)




add_library(${PROJECT_NAME}
src/collision_detection_controller.cpp
src/collision_detection_controller.cpp
src/jaesug_controller.cpp
#src/position_joint_space_slam_test_controller.cpp
#src/impedance_controller.cpp
#src/test_controller1.cpp
src/position_task_space_controller.cpp
#src/position_task_space_controller_sy_dual_pin.cpp
#src/position_task_space_controller_sy_circle.cpp
src/position_joint_space_controller.cpp
#src/position_joint_space_controller_tasktransition.cpp
#src/position_joint_space_controller_je.cpp
#src/position_joint_dob_controller.cpp
#src/velocity_task_space_controller.cpp
#src/velocity_joint_space_controller.cpp
#src/velocity_joint_space_controller_dob.cpp
src/suhan_controller.cpp
#src/hw1_controller.cpp
src/torque_joint_space_controller.cpp
#src/torque_joint_space_controller_sy.cpp
#src/torque_joint_space_controller_spiral.cpp
src/torque_joint_space_controller_dual_spiral.cpp
#src/torque_joint_space_controller_screw.cpp
#src/torque_joint_space_controller_screw2.cpp
#src/torque_joint_space_controller_with_torque_ext.cpp
#src/teaching_detecting_dob.cpp
#src/teaching_detecting_recording.cpp
src/torque_joint_space_controller_rrt.cpp
#src/torque_joint_space_controller_three_pin.cpp
#src/torque_joint_space_controller_sy_three_pin.cpp
#src/torque_joint_space_controller_sy_circle.cpp
src/torque_joint_space_controller_assembly_strategy.cpp
#src/torque_joint_space_controller_stefan_gravity.cpp
src/torque_joint_space_controller_side_chair.cpp
src/torque_joint_space_controller_place.cpp
src/torque_joint_space_controller_sy_dual_a.cpp
src/torque_joint_space_controller_sy_dual_pin.cpp
#src/torque_joint_space_controller_sy_press.cpp
src/torque_joint_space_controller_sy_startpoint.cpp
src/torque_joint_space_controller_revolve.cpp
src/torque_joint_space_controller_realsense.cpp
src/torque_joint_space_controller_hip.cpp
#src/torque_joint_space_controller_triple.cpp
src/torque_joint_space_controller_fuzzy.cpp
src/torque_joint_space_controller_drill.cpp
src/robot_model.cpp
src/torque_joint_space_controller_joint_test.cpp
src/position_joint_space_controller_joint_test.cpp
src/velocity_joint_space_controller.cpp
)

add_dependencies(advanced_robotics_franka_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
    ${RBDL_LIBRARY}

)
#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dyros_mobile_manipulator_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
