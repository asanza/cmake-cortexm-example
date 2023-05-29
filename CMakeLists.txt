
#   Copyright (c) 2023 Diego Asanza <f.asanza@gmail.com>
#   Created on Mon May 29 2023
 

# SPDX-License-Identifier: BSD 3-Clause 

cmake_minimum_required(VERSION 3.20)

# Enable export of compilation database for further analysis
# This creates a compile_comands.json file in the build directory that
# can be used by visual studio and cppcheck for code completion and
# build analysis.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the path to custom modules
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# This is the toolchain file, we use the arm-none-eabi toolchain
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_MODULE_PATH}/arm-gcc-toolchain.cmake)

# We use python to run the unit tests
find_package(Python COMPONENTS Interpreter REQUIRED)

project(BLINKY LANGUAGES C ASM DESCRIPTION "Blinky!")

enable_testing()

# the compiler options
add_compile_options(
  -mcpu=cortex-m3
  -mthumb
  -O0
  -ggdb
  -fdiagnostics-color=always
  -ffunction-sections
  -fdata-sections
  -Wall
)

# and the linker options
add_link_options(
    -mcpu=cortex-m3
    -mthumb
)

add_subdirectory(src)
add_subdirectory(test)