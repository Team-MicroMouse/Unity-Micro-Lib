cmake_minimum_required(VERSION 3.30)
project(Unity_Link_Test)

set(CMAKE_CXX_STANDARD 20)

add_library(Unity_Link_Test SHARED library.cpp
        com/com.cpp
        interface/interface.cpp
)
