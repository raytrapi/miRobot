cmake_minimum_required(VERSION 3.0.0)

project(controlador1)
set (CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )

include_directories(
   src
   ${OpenCV_INCLUDE_DIRS}
   /opt/ros/lunar/include
   /usr/include/gazebo-9
   /usr/include/sdformat-6.2
   /usr/include/ignition/math4
)

add_executable(controlador1 src/main.cpp)
target_link_libraries( controlador1 ${OpenCV_LIBS} /opt/ros/melodic/lib/libroscpp.so /opt/ros/melodic/lib/libroslib.so
	/opt/ros/melodic/lib/librosconsole.so /opt/ros/melodic/lib/libroscpp_serialization.so 
	/opt/ros/melodic/lib/libimage_transport.so /opt/ros/melodic/lib/librostime.so
	/usr/lib/x86_64-linux-gnu/libgazebo.so /usr/lib/x86_64-linux-gnu/libgazebo_common.so 
	/usr/lib/x86_64-linux-gnu/libignition-math4.so
	${Boost_LIBRARIES} /usr/lib/x86_64-linux-gnu/libboost_system.so plugin dl
	)


add_library(plugin "SHARED"  src/plugin.cpp src/algoritmoPlugin.cpp)
set_target_properties(plugin PROPERTIES
	COMPILE_DEFINITIONS PLUGIN_EXPORTS
    PUBLIC_HEADER src/plugin.h )

target_link_libraries( plugin  ${OpenCV_LIBS})

add_subdirectory(algoritmos)