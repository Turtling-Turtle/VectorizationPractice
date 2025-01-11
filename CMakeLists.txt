cmake_minimum_required(VERSION 3.28)
project(VectorizationPractice C)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_FLAGS "-Ofast -march=native")

# https://www.youtube.com/watch?v=_5wbp_bD5HA
include(FetchContent)
set(RAYLIB_VERSION 5.5)
FetchContent_Declare(
        raylib
        URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_VERSION}.tar.gz
        FIND_PACKAGE_ARGS ${RAYLIB_VERSION}
)
set(BUILD_EXAMPLES OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(raylib)

add_executable(VectorizationPractice main.c
        PhysicsObjects.h
        PhysicsObjects.c)

target_link_libraries(VectorizationPractice raylib)