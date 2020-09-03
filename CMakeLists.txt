cmake_minimum_required(VERSION 3.0)
project(expl_path_planner)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package(CATKIN_DEPENDS)

add_executable(explPlanner src/main.cc)
