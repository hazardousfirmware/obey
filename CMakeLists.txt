cmake_minimum_required(VERSION 3.20)

project(obey)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++23 -Werror")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror")

add_executable(obey
        CAN.cpp
        ISO15765.cpp
        main.cpp
)

add_executable(obdsim
        obdsim.c
)
