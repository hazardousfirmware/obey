cmake_minimum_required(VERSION 3.0)

project(obey)

add_executable(obey
        CAN.cpp
        ISO15765.cpp
        main.cpp
)

add_executable(obdsim
        obdsim.c
)
