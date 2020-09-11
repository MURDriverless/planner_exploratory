cmake_minimum_required(VERSION 2.8.3)
project(planner_exploratory)
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs
    )

add_definitions(-std=c++14)

include_directories(${catkin_INCLUDE_DIRS})
catkin_package(CATKIN_DEPENDS)

add_executable(main src/main.cpp)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++14; Wall; Werror")

add_library(path_point src/path_point.cpp)
add_library(cone src/cone.cpp)
add_library(path_planner src/path_planner.cpp)
add_library(node src/node.cpp)

target_link_libraries(main ${catkin_LIBRARIES} node path_point cone path_planner)

