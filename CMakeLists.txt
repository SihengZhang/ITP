cmake_minimum_required(VERSION 3.1)
project(distances)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

### libIGL options: choose between header only and compiled static library
option(LIBIGL_USE_STATIC_LIBRARY     "Use libigl as static library" OFF)
option(LIBIGL_GLFW       "Use GLFW"           ON)

include(libigl)

# Add your project files
include_directories("include/")
if(USE_SOLUTION)
  file(GLOB SRCFILES solution/*.cpp)
else()
  file(GLOB SRCFILES src/*.cpp)
endif()

add_library(core ${SRCFILES}
        src/get_dijkstra_edges_path.cpp
        include/get_dijkstra_edges_path.h
        src/signpost_data_structure.cpp
        include/signpost_data_structure.h
        include/check_mesh_manifold_and_orientation.h
        src/check_mesh_manifold_and_orientation.cpp
)
target_link_libraries(core igl::core igl::glfw )

add_executable(distances "main.cpp")
target_link_libraries(distances core igl::core igl::opengl igl::glfw )
