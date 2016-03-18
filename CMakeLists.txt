cmake_minimum_required(VERSION 3.4)

set(PROJECT robotpi_server)

project(${PROJECT})

find_library( bcm2835_lib bcm2835)

#include_directories("/usr/include")

SET(CMAKE_CXX_FLAGS "-pthread -lwiringPi -lbcm2835")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lwiringPi -lbcm2835")

#target_link_libraries(minimu ${WIRINGPI_LIBRARIES})
#target_link_libraries(minimu ${CMAKE_THREAD_LIBS_INIT})

set(SOURCE_FILES
    robotpi_socket.c)

add_executable(${PROJECT}
        ${SOURCE_FILES})
