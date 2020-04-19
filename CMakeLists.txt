cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(Hough)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# if(NOT CMAKE_BUILD_TYPE )
 set(CMAKE_BUILD_TYPE Debug)
# endif()

# set( CMAKE_CXX_FLAGS_RELEASE "-g -O3")
#set( CMAKE_CXX_FLAGS_DEBUG "-O1") # Optimizing for maximum speed
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3") # For Linux

# Add needed packages
find_package( OpenCV REQUIRED )
include_directories(include)
#include_directories( ${OpenCV_INCLUDE_DIRS} )

# add the MathFunctions library
#add_subdirectory(src)

# add the executable
add_executable(Hough src/hough.cc)

target_sources(Hough PRIVATE src/filters.cc src/collisions.cc)
target_link_libraries(Hough PUBLIC ${OpenCV_LIBS})

# add the binary tree to the search path for include files
target_include_directories(Hough PUBLIC)