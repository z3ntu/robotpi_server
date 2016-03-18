cmake_minimum_required(VERSION 3.4)
project(robotpi_server)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lwiringPi -lbcm2835")

set(SOURCE_FILES
    README.md
    robotpi_socket.c)

add_executable(robotpi_server ${SOURCE_FILES})
