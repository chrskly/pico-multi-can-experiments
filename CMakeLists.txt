cmake_minimum_required(VERSION 3.22)

include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(multican)

pico_sdk_init()

add_executable(multican
        multican.c
        can.h
        )


target_link_libraries(multican pico_stdlib hardware_spi)

pico_add_extra_outputs(multican)

#example_auto_set_url(multican)

