cmake_minimum_required(VERSION 3.25)
project(scrollview C)

set(CMAKE_C_STANDARD 11)

include_directories(sv/inc)

add_executable(scrollview
        sv/inc/scrollview.h
        sv/src/scrollview.c)

target_compile_options(scrollview PRIVATE
        -Weverything -g3 -fwrapv -fno-omit-frame-pointer
        $<$<CONFIG:Debug>: -Og>
        $<$<CONFIG:Release>:-O3>)