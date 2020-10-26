cmake_minimum_required(VERSION 2.8.3)
project(planner_exploratory)
add_definitions(-std=c++14)
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs
    message_filters
    nav_msgs
    geometry_msgs
    mur_common
    )

set (CMAKE_CXX_FLAGS_DEBUG "-g")
set (CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Boost COMPONENTS math)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
catkin_package(CATKIN_DEPENDS)

add_executable(planner src/main.cpp)

add_library(node src/node.cpp)
add_library(path_planner src/path_planner.cpp)
add_library(cone src/cone.cpp)
add_library(path_point src/path_point.cpp)

target_link_libraries(planner ${catkin_LIBRARIES} node path_planner cone path_point) 