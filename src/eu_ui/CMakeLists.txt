cmake_minimum_required(VERSION 3.0.2)
project(eu_ui)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/execution-charset:utf-8>")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#    qt_build
    rviz
    roscpp
    std_msgs
    moveit_msgs
    moveit_ros_perception
    moveit_ros_planning_interface
    )
find_package(Qt5 REQUIRED Core Widgets Xml Sql PrintSupport)
set(QT_LIBRARIES Qt5::Widgets Qt5::Xml Qt5::Sql Qt5::PrintSupport)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES eu_ui
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_subdirectory(eu_control)
