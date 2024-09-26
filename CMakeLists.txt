# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(BOARD nslog)


# OOT Drivers
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ns_mano_fw)

target_include_directories(app PRIVATE src)
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/mano.c)


