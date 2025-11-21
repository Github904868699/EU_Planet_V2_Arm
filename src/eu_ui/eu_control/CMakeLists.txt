include_directories(
    planning
    motorcontrol
    coordinate
    include
    ../eu_ui/appconfig
    ../../../devel/include/eu_msgs
    ${catkin_INCLUDE_DIRS}
    )
file(GLOB_RECURSE QT_MOC RELATIVE
    ${CMAKE_CURRENT_SOURCE_DIR} *.h
    ../eu_ui/appconfig/*.h
    )
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp
    ../eu_ui/appconfig/*.cpp
    )
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
#message(${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/eu_ui/eu_arm.so)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
add_library(eu_arm ${QT_SOURCES} ${QT_MOC_HPP})
target_link_libraries(eu_arm ${QT_LIBRARIES} ${catkin_LIBRARIES})
