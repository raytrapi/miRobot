cmake_minimum_required(VERSION 3.0.0)
#include(GenerateExportHeader)

project(control_remoto)



set(CMAKE_AUTOMOC ON)
find_package(Qt5Core REQUIRED)
find_package( Qt5Widgets )
include_directories(
   src
   ${Qt5Core_INCLUDE_DIRS}
   /opt/ros/kinetic/include
)
set(CMAKE_AUTOMOC ON)

add_definitions(${Qt5Core_DEFINITIONS})



#add_definitions(${Qt5Core_DEFINITIONS})
add_executable(control_remoto src/main.cc src/mainDialog.cc )
target_link_libraries( control_remoto  ${Qt5Widgets_LIBRARIES} /opt/ros/kinetic/lib/libroscpp.so /opt/ros/kinetic/lib/libroslib.so
/opt/ros/kinetic/lib/librosconsole.so /opt/ros/kinetic/lib/libroscpp_serialization.so)