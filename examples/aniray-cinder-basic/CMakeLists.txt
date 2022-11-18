# CMakeLists.txt: CMake configuration for example program aniray-cinder-basic
#
# Created by Perry Naseck on 2022-11-17.
#
# This file is a part of Aniray
# https://github.com/HypersonicED/aniray
#
# Copyright (c) 2022, Hypersonic
# Copyright (c) 2022, Perry Naseck
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
set(CMAKE_VERBOSE_MAKEFILE ON)

project(example-aniray-cinder-basic)

set (CMAKE_CXX_STANDARD 17)

# TODO: should not be required here
find_package(Boost 1.74.0 REQUIRED COMPONENTS log)

get_filename_component(CINDER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/Cinder" ABSOLUTE)
get_filename_component(ANIRAY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(APP_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
set(aniray_DIR "${ANIRAY_PATH}/_builds/${CMAKE_BUILD_TYPE}")
find_package(aniray REQUIRED)

add_compile_options(-pg)

include("${CINDER_PATH}/proj/cmake/modules/cinderMakeApp.cmake")

ci_make_app(
    APP_NAME    "ExampleAnirayCinderBasic"
    CINDER_PATH ${CINDER_PATH}
    SOURCES     ${APP_PATH}/src/main.cpp ${APP_PATH}/src/RotationAnimation.cpp
    ASSETS_PATH ${APP_PATH}/assets
    LIBRARIES   aniray::aniray
)
