cmake_minimum_required(VERSION 3.28)
project(VectorizationPractice C)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_FLAGS "-O1 -march=native")

add_executable(VectorizationPractice main.c)
