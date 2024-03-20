cmake_minimum_required(VERSION 3.26)
project(graph_cut_final)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(CGAL REQUIRED)
add_executable(graph_cut_final main.cpp)
target_link_libraries(graph_cut_final CGAL::CGAL)
