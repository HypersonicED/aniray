# CMakeLists.txt: CMake configuration for Aniray
#
# Created by Perry Naseck on 2022-09-30.
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
project(aniray LANGUAGES CXX)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
    YES
    CACHE BOOL "Export all symbols")

option(ANIRAY_WITHOUT_OLA "Build without Open Lighting Architecture" NO)
option(ANIRAY_WITHOUT_MODBUS "Build without Modbus" NO)
option(ANIRAY_WITH_LINT "Run clang-tidy and iwyu" NO)

find_package(Boost 1.74.0 REQUIRED COMPONENTS log)
find_package(OpenSSL 1.1.0 REQUIRED)

if(ANIRAY_WITH_LINT)
  find_program(IWYU_EXE NAMES include-what-you-use iwyu REQUIRED)
  find_program(CLANG_TIDY_EXE NAMES "clang-tidy" "clang-tidy-13" "clang-tidy-12"
                                    "clang-tidy-11" REQUIRED)
endif()

if(NOT ANIRAY_WITHOUT_OLA)
  find_package(PkgConfig)
  pkg_check_modules(LIBOLA libola>=0.10.8 REQUIRED)
endif()

if(NOT ANIRAY_WITHOUT_MODBUS)
  pkg_check_modules(LIBMODBUS libmodbus>=3.1.6 REQUIRED)
endif()

add_library(
  ${CMAKE_PROJECT_NAME}
  include/aniray/DMXAddr.hpp
  include/aniray/Geometry.hpp
  include/aniray/IOInterface.hpp
  include/aniray/IOInterfaceModbus.hpp
  include/aniray/Node.hpp
  include/aniray/NodeAnimation.hpp
  include/aniray/NodeArray.hpp
  include/aniray/NodeArrayOutput.hpp
  include/aniray/NodeArraySampler.hpp
  include/aniray/PeriodicThread.hpp
  lib/fast-cpp-csv-parser/csv.h
  src/DMXAddr.cpp
  src/IOInterface.cpp
  src/PeriodicThread.cpp)
add_library("${CMAKE_PROJECT_NAME}::${CMAKE_PROJECT_NAME}" ALIAS "${CMAKE_PROJECT_NAME}")

target_compile_features(${CMAKE_PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Werror)
set_target_properties(
  ${CMAKE_PROJECT_NAME}
  PROPERTIES LINKER_LANGUAGE CXX)
              # CXX_EXTENSIONS OFF)

if(NOT ANIRAY_WITHOUT_OLA)
  # Private to avoid double linking https://stackoverflow.com/a/34533415
  target_sources(${CMAKE_PROJECT_NAME} PRIVATE
  include/aniray/NodeArrayOutputOLA.hpp
  src/NodeArrayOutputOLA.cpp)
endif()
if(NOT ANIRAY_WITHOUT_MODBUS)
  # Private to avoid double linking https://stackoverflow.com/a/34533415
  target_sources(${CMAKE_PROJECT_NAME} PRIVATE include/aniray/IOInterfaceModbus.hpp)
endif()

if(ANIRAY_WITH_LINT)
  # Private to avoid double linking https://stackoverflow.com/a/34533415
  target_sources(${CMAKE_PROJECT_NAME} PRIVATE
  src/lintHelpers/Geometry.cpp
  src/lintHelpers/Node.cpp
  src/lintHelpers/NodeAnimation.cpp
  src/lintHelpers/NodeArray.cpp
  src/lintHelpers/NodeArrayOutput.cpp
  src/lintHelpers/NodeArraySampler.cpp)
  if(NOT ANIRAY_WITHOUT_MODBUS)
    # Private to avoid double linking https://stackoverflow.com/a/34533415
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE src/lintHelpers/IOInterfaceModbus.cpp)
  endif()
  set_target_properties(
    ${CMAKE_PROJECT_NAME}
    PROPERTIES
    CXX_INCLUDE_WHAT_YOU_USE "${IWYU_EXE};-Xiwyu;--cxx17ns;-Xiwyu;--mapping_file=${CMAKE_CURRENT_SOURCE_DIR}/lib/boost-1.75-all.imp"
    CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
endif()

target_include_directories(
  ${CMAKE_PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_include_directories(
  ${CMAKE_PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/fast-cpp-csv-parser>
         $<INSTALL_INTERFACE:include/fast-cpp-csv-parser>)

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${LIBOLA_INCLUDE_DIRS})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${OPENSSL_INCLUDE_DIR})
target_link_libraries(${CMAKE_PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${CMAKE_PROJECT_NAME} ${LIBOLA_LIBRARIES})
target_link_libraries(${CMAKE_PROJECT_NAME} ${OPENSSL_LIBRARIES})

set(TARGETS_EXPORT_NAME "${CMAKE_PROJECT_NAME}Config")
install(
  TARGETS ${CMAKE_PROJECT_NAME}
  EXPORT               "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION  "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION  "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION  "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
export(
  TARGETS ${CMAKE_PROJECT_NAME}
  NAMESPACE ${CMAKE_PROJECT_NAME}::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${TARGETS_EXPORT_NAME}.cmake"
)
install(
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION "${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME}/cmake"
  NAMESPACE ${CMAKE_PROJECT_NAME}::
)
