cmake_minimum_required(VERSION 3.16)

project(mesh_adapt
    VERSION 0.1.0
    DESCRIPTION "Adaptive remeshing core library (2D/3D FEM/ALE)"
    LANGUAGES CXX
)

# -----------------------------
# C++ Standard
# -----------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -----------------------------
# Library target
# -----------------------------
add_library(mesh_adapt 
    src/mesh_adapt/core/Mesh2D.cpp
    src/mesh_adapt/geometry/Polyline2D.cpp
    src/mesh_adapt/boundary/Boundary2D.cpp
    src/mesh_adapt/boundary/Delaunay2D.cpp
)

# Public include directory
target_include_directories(mesh_adapt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


# Optional warnings (only if used as main project)
if (PROJECT_IS_TOP_LEVEL)
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Example executable
# -----------------------------
add_executable(mesh-adapt
    src/main.cpp
)

add_executable(example
    src/example.cpp
)
target_include_directories(mesh-adapt PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(example PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(example PRIVATE mesh_adapt)

target_link_libraries(mesh-adapt PRIVATE mesh_adapt)

# -----------------------------
# Install support (optional)
# -----------------------------
install(DIRECTORY include/
    DESTINATION include
)

install(TARGETS mesh_adapt
    EXPORT mesh_adaptTargets
)

