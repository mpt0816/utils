cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11 -g)

project(utils)

include_directories(src)

add_executable(utils src/main.cpp)