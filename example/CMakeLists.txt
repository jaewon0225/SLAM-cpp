cmake_minimum_required(VERSION 3.20)
project(example)
add_executable(example example.cpp)
target_link_libraries(example cpp_slam)