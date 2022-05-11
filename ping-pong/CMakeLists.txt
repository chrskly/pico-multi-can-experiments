cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pingPong)

#enable_language( C CXX ASM )

pico_sdk_init()

add_executable(pingPong
        mcp2515/mcp2515.cpp
        pingPong.cpp
        mcp2515/can.h
        mcp2515/mcp2515.h
        )

pico_add_extra_outputs(pingPong)

target_link_libraries(pingPong pico_stdlib hardware_spi)

