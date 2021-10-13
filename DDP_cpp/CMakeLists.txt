cmake_minimum_required(VERSION 3.0.0)
project(DDP VERSION 1.0.0.0)
set(CMAKE_CXX_STANDARD 11)

add_compile_options(-Wall -Wextra -pedantic -Werror)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -std=c++11")
set(CMAKE_BUILD_TYPE Debug)
