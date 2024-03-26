cmake_minimum_required(VERSION 3.18)
project(graph_cut_final)

set(CMAKE_CXX_STANDARD 17)
set(Qt5_DIR "/usr/local/opt/qt@5/lib/cmake/Qt5")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(CGAL REQUIRED COMPONENTS Core Qt5)
add_definitions(-DCGAL_USE_BASIC_VIEWER)
add_executable(graph_cut_final main.cpp
        include/utils.h)
target_link_libraries(graph_cut_final CGAL::CGAL CGAL::CGAL_Basic_viewer)
