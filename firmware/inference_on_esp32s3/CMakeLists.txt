# Standalone ESP-IDF project (no shared top-level build, like the other apps in firmware/).
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(inference_on_esp32s3)
