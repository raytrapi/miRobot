cmake_minimum_required(VERSION 3.0.0)
 
project(teVeo)
set (CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )
find_package( gazebo REQUIRED )
#find_package(  REQUIRED )

include_directories(
   src
   ${OpenCV_INCLUDE_DIRS}
   /opt/ros/lunar/include
   ${GAZEBO_INCLUDE_DIRS}
)

add_executable(teVeo src/camara_main.cpp src/camara.cpp)
target_link_libraries( teVeo ${OpenCV_LIBS} /opt/ros/melodic/lib/libroscpp.so /opt/ros/melodic/lib/libroslib.so
	/opt/ros/melodic/lib/librosconsole.so /opt/ros/melodic/lib/libroscpp_serialization.so 
	/opt/ros/melodic/lib/libimage_transport.so /opt/ros/melodic/lib/librostime.so
	/usr/lib/x86_64-linux-gnu/libgazebo.so /usr/lib/x86_64-linux-gnu/libgazebo_common.so
	/usr/lib/x86_64-linux-gnu/libignition-math4.so
	${Boost_LIBRARIES} /usr/lib/x86_64-linux-gnu/libboost_system.so
	)
#/opt/ros/lunar/lib/libcamera_info_manager.so   /opt/ros/melodi/lib/libcpp_common.so 