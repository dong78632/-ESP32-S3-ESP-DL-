cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s3_edge_security)

if(NOT IDF_TARGET STREQUAL "esp32s3")
    message(FATAL_ERROR "This project requires idf.py set-target esp32s3")
endif()
