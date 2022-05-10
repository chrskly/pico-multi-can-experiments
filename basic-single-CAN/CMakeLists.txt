cmake_minimum_required(VERSION 3.22)

include(pico_sdk_import.cmake)
#include(pico_extras_import.cmake)

project(basicSingle)

pico_sdk_init()

add_executable(basicSingle
        mcp2515/mcp2515.cpp
        basicSingle.cpp
        mcp2515/can.h
        mcp2515/mcp2515.h
        )

target_link_libraries(basicSingle pico_stdlib hardware_spi)

pico_add_extra_outputs(basicSingle)

