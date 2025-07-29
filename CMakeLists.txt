# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)
include(${CMAKE_CURRENT_LIST_DIR}/toolchain/esp-idf/tools/cmake/project.cmake)
project(ESP32-FPGA-MatrixPanel)