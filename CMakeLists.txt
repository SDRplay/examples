cmake_minimum_required(VERSION 3.6)
project(examples)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        play_sdr.c)

add_executable(play_sdr ${SOURCE_FILES})
target_link_libraries (play_sdr pthread m mirsdrapi-rsp)
