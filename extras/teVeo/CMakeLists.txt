cmake_minimum_required(VERSION 3.0.0)

project(teVeo)
set (CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )

include_directories(
   src
   ${OpenCV_INCLUDE_DIRS}
   /opt/ros/lunar/include
   /usr/include/gazebo-7
   /usr/include/sdformat-4.4
   /usr/include/ignition/math2 
)

add_executable(teVeo src/camara_main.cpp src/camara.cpp)
target_link_libraries( teVeo ${OpenCV_LIBS} /opt/ros/lunar/lib/libroscpp.so /opt/ros/lunar/lib/libroslib.so
	/opt/ros/lunar/lib/librosconsole.so /opt/ros/lunar/lib/libroscpp_serialization.so 
	/opt/ros/lunar/lib/libimage_transport.so /opt/ros/lunar/lib/librostime.so
	/usr/lib/x86_64-linux-gnu/libgazebo.so /usr/lib/x86_64-linux-gnu/libgazebo_common.so /usr/lib/x86_64-linux-gnu/libgazebo_math.so
	${Boost_LIBRARIES} /usr/lib/x86_64-linux-gnu/libboost_system.so
	)
#/opt/ros/lunar/lib/libcamera_info_manager.so   /opt/ros/lunar/lib/libcpp_common.so 