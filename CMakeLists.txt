cmake_minimum_required(VERSION 3.18)
project(graph_cut_final)

set(CMAKE_CXX_STANDARD 17)
set(Qt5_DIR "/usr/local/opt/qt@5/lib/cmake/Qt5")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
find_package(CGAL REQUIRED COMPONENTS Core Qt5)
#add_definitions(-DCGAL_USE_BASIC_VIEWER)

add_library(IBFS graph_cut/IBFS.h graph_cut/IBFS.cpp)

add_executable(graph_cut_2d main.cpp)

target_link_libraries(graph_cut_2d IBFS CGAL::CGAL CGAL::CGAL_Basic_viewer)
