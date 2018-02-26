cmake_minimum_required(VERSION 3.0.0)

project(controlador1)
set (CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )

include_directories(
   src
   ${OpenCV_INCLUDE_DIRS}
   /opt/ros/kinetic/include
   /usr/include/gazebo-7
   /usr/include/sdformat-4.4
   /usr/include/ignition/math2
)

add_executable(controlador1 src/main.cpp)
target_link_libraries( controlador1 ${OpenCV_LIBS} /opt/ros/kinetic/lib/libroscpp.so /opt/ros/kinetic/lib/libroslib.so
	/opt/ros/kinetic/lib/librosconsole.so /opt/ros/kinetic/lib/libroscpp_serialization.so 
	/opt/ros/kinetic/lib/libimage_transport.so /opt/ros/kinetic/lib/librostime.so
	/usr/lib/x86_64-linux-gnu/libgazebo.so /usr/lib/x86_64-linux-gnu/libgazebo_common.so /usr/lib/x86_64-linux-gnu/libgazebo_math.so
	${Boost_LIBRARIES} /usr/lib/x86_64-linux-gnu/libboost_system.so plugin dl
	)


add_library(plugin "SHARED"  src/plugin.cpp src/algoritmoPlugin.cpp)
set_target_properties(plugin PROPERTIES
	COMPILE_DEFINITIONS PLUGIN_EXPORTS
    PUBLIC_HEADER src/plugin.h )

target_link_libraries( plugin  ${OpenCV_LIBS})

add_subdirectory(algoritmos)