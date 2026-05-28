# Copyright (c) 2026 Meta Platforms
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i3c_loopback_test)

target_sources(app PRIVATE
    src/common.c
    src/test_attach.c
    src/test_ccc.c
    src/test_configure.c
    src/test_daa.c
    src/test_hdr_ddr.c
    src/test_ibi.c
    src/test_target_api.c
    src/test_xfer.c
)

target_include_directories(app PRIVATE src)
