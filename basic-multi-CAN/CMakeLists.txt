cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(basicMulti)

#enable_language( C CXX ASM )

pico_sdk_init()

add_executable(basicMulti
        mcp2515/mcp2515.cpp
        basicMulti.cpp
        mcp2515/can.h
        mcp2515/mcp2515.h
        )

pico_add_extra_outputs(basicMulti)

target_link_libraries(basicMulti pico_stdlib hardware_spi)

