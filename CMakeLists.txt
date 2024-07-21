cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(cpp_slam VERSION 1.0.0 LANGUAGES CXX)

include_directories(src
                    include
)

add_library(cpp_slam 
            src/slam.cpp
            src/threadpool.cpp
            src/timer.cpp
)

target_include_directories(cpp_slam
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_INSTALL_INCLUDEDIR}
)

set(PUBLIC_HEADERS
    slam.hpp
    threadpool.hpp
    timer.hpp
)

set_target_properties(cpp_slam PROPERTIES PUBLIC_HEADER "${PUBLIC_HEADERS}")

# Find the Eigen3 package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(cpp_slam Eigen3::Eigen)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

install(TARGETS cpp_slam DESTINATION lib
        PUBLIC_HEADER DESTINATION include
)

# -----------------------------------------------------------------
add_subdirectory( example )