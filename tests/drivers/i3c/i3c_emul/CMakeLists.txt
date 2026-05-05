# Copyright (c) 2026 Ryan McClelland
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i3c_emul)

target_sources(app PRIVATE
    src/test_core.c
    src/test_target_emul.c
)
target_sources_ifdef(CONFIG_I3C_USE_IBI app PRIVATE src/test_ibi.c)
target_sources_ifdef(CONFIG_I3C_CALLBACK app PRIVATE src/test_async.c)
target_sources_ifdef(CONFIG_I3C_TARGET app PRIVATE src/test_target.c)
target_include_directories(app PRIVATE src)
