cmake_minimum_required(VERSION 3.10)

project(AVParse)
enable_language(CXX)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

add_executable(ts_parser ts_dec.cc)