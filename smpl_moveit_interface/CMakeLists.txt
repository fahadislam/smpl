cmake_minimum_required(VERSION 2.8.3)
project(smpl_moveit_interface)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(SMPL_MOVEIT_INTERFACE_QT5 "Build RViz plugin against Qt5" ON)

find_package(Boost REQUIRED)

if(${SMPL_MOVEIT_INTERFACE_QT5})
    find_package(Qt5Widgets REQUIRED)
    find_package(Qt5Core REQUIRED)
else()
    find_package(Qt4 REQUIRED)
    include(${QT_USE_FILE})
endif()

find_package(
    catkin
    REQUIRED
    COMPONENTS
        actionlib
        eigen_conversions
        geometric_shapes
        interactive_markers
        moveit_core
        moveit_msgs
        moveit_ros_planning
        roscpp
        rviz
        smpl_ros
        sbpl_collision_checking
        sensor_msgs
        pluginlib
        visualization_msgs)

find_package(smpl REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES smpl_moveit_robot_model move_group_command_panel_plugin
    CATKIN_DEPENDS
        eigen_conversions
        geometric_shapes
        smpl_ros
        sbpl_collision_checking
        roscpp
    DEPENDS
        smpl)

add_definitions(-DSV_PACKAGE_NAME="smpl_moveit_interface")

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

add_library(smpl_moveit_robot_model src/planner/moveit_robot_model.cpp)

################################################
# Build move_group_command_panel_plugin plugin #
################################################

set(MOVE_GROUP_COMMAND_PANEL_MOC_HEADERS
    include/smpl_moveit_interface/interface/joint_variable_command_widget.h
    include/smpl_moveit_interface/interface/robot_command_model.h
    include/smpl_moveit_interface/interface/ik_command_interactive_marker.h
    include/smpl_moveit_interface/interface/teleop_command.h
    src/interface/move_group_command_model.h
    src/interface/move_group_command_panel.h)

if(SMPL_MOVEIT_INTERFACE_QT5)
    qt5_wrap_cpp(MOVE_GROUP_COMMAND_PANEL_MOC_SOURCES ${MOVE_GROUP_COMMAND_PANEL_MOC_HEADERS})
else()
    qt4_wrap_cpp(MOVE_GROUP_COMMAND_PANEL_MOC_SOURCES ${MOVE_GROUP_COMMAND_PANEL_MOC_HEADERS})
endif()

if(SMPL_MOVEIT_INTERFACE_QT5)
    set(MOVE_GROUP_COMMAND_PANEL_QT_LIBRARIES Qt5::Core Qt5::Widgets)
else()
    set(MOVE_GROUP_COMMAND_PANEL_QT_LIBRARIES ${QT_LIBRARIES})
endif()

add_library(
    move_group_command_panel_plugin
    ${MOVE_GROUP_COMMAND_PANEL_MOC_SOURCES}
    src/interface/ik_command_interactive_marker.cpp
    src/interface/teleop_command.cpp
    src/interface/joint_variable_command_widget.cpp
    src/interface/move_group_command_model.cpp
    src/interface/move_group_command_panel.cpp
    src/interface/robot_command_model.cpp
    src/interface/utils.cpp)
target_link_libraries(
    move_group_command_panel_plugin
    ${MOVE_GROUP_COMMAND_PANEL_QT_LIBRARIES}
    ${catkin_LIBRARIES}
    smpl::smpl)

# Ad-hoc handling of API differences between indigo and kinetic distributions
if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    set(COLLISION_DETECTION_SBPL_ROS_VERSION COLLISION_DETECTION_SBPL_ROS_KINETIC)
else()
#elseif ("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    set(COLLISION_DETECTION_SBPL_ROS_VERSION COLLISION_DETECTION_SBPL_ROS_INDIGO)
endif()

#########################################
# Build collision_detection_sbpl plugin #
#########################################

add_library(
    collision_detection_sbpl
    src/collision/collision_plugin_sbpl.cpp
    src/collision/collision_common_sbpl.cpp
    src/collision/collision_detector_allocator_sbpl.cpp
    src/collision/collision_robot_sbpl.cpp
    src/collision/collision_world_sbpl.cpp)

target_include_directories(collision_detection_sbpl PRIVATE src)

target_compile_definitions(
    collision_detection_sbpl
    PRIVATE
    -DCOLLISION_DETECTION_SBPL_ROS_VERSION=${COLLISION_DETECTION_SBPL_ROS_VERSION})

target_link_libraries(collision_detection_sbpl ${catkin_LIBRARIES} smpl::smpl)

###########################################
# Build moveit_sbpl_planner_plugin plugin #
###########################################

add_library(
    moveit_sbpl_planner_plugin
    src/planner/planner_family_manager.cpp
    src/planner/sbpl_planner_manager.cpp
    src/planner/sbpl_planning_context.cpp
    src/planner/moveit_collision_checker.cpp)

target_compile_definitions(
    moveit_sbpl_planner_plugin
    PRIVATE
    -DCOLLISION_DETECTION_SBPL_ROS_VERSION=${COLLISION_DETECTION_SBPL_ROS_VERSION})

target_include_directories(moveit_sbpl_planner_plugin PRIVATE src)

target_link_libraries(
    moveit_sbpl_planner_plugin
    collision_detection_sbpl
    smpl_moveit_robot_model
    ${catkin_LIBRARIES}
    smpl::smpl)

###########
# Install #
###########

install(
    DIRECTORY include/smpl_moveit_interface
    DESTINATION include
    FILES_MATCHING PATTERN "*.h")

install(
    FILES
        sbpl_interface_plugin_description.xml
        collision_detection_sbpl_plugin_description.xml
        move_group_command_panel_plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    TARGETS
        smpl_moveit_robot_model
        collision_detection_sbpl
        moveit_sbpl_planner_plugin
        move_group_command_panel_plugin
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
