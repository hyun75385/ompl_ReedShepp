cmake_minimum_required(VERSION 2.8.3)
project(omplrscurve)


## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    )

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/omplrscurve.cpp
    )
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/omplrscurve.h
    )
# add_library(HYAS ${SOURCES} ${HEADERS})

## Declare a catkin package
catkin_package()

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)


include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OMPL_INCLUDE_DIRS})
include_directories(include include)

add_executable(omplrscurve src/omplrscurve.cpp)
target_link_libraries(omplrscurve ${catkin_LIBRARIES})
target_link_libraries(omplrscurve ${OMPL_LIBRARIES})
