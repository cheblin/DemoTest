cmake_minimum_required(VERSION 3.17)
project(InCPP)

set(CMAKE_CXX_STANDARD 14)

include_directories(GroundControl)

add_executable(InCPP
        GroundControl/AdHoc.c
        GroundControl/AdHoc.h
        GroundControl/config_.h
        GroundControl/Demo_.cpp
        GroundControl/GroundControl.cpp
        GroundControl/GroundControl.hpp
        GroundControl/Test_.cpp
        GroundControl/Utils_.hpp)
