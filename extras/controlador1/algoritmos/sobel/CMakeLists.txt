#set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/bin)
find_package( OpenCV REQUIRED )
include_directories(
   src
   ${OpenCV_INCLUDE_DIRS}
   ../src
)
#link_directories(${CMAKE_BINARY_DIR}/bin/)

add_library(sobel "SHARED" src/sobel.cpp)

target_link_libraries( sobel ${OpenCV_LIBS} plugin)
