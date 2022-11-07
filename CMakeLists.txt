cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(aniray LANGUAGES CXX)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
    YES
    CACHE BOOL "Export all symbols")

option(ANIRAY_WITHOUT_OLA "Build without Open Lighting Architecture" NO)
option(ANIRAY_WITH_LINT "Run clang-tidy and iwyu" NO)

add_definitions(-DBOOST_LOG_DYN_LINK)
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

add_library(
  ${CMAKE_PROJECT_NAME}
  include/aniray/DMXAddr.hpp
  include/aniray/Geometry.hpp
  include/aniray/InputInterface.hpp
  include/aniray/Node.hpp
  include/aniray/NodeAnimation.hpp
  include/aniray/NodeArray.hpp
  include/aniray/NodeArrayOutput.hpp
  include/aniray/NodeArraySampler.hpp
  include/aniray/PeriodicThread.hpp
  src/DMXAddr.cpp
  src/InputInterface.cpp
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
  target_sources(${CMAKE_PROJECT_NAME} PRIVATE include/aniray/NodeArrayOutputOLA.hpp)
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
  if(NOT ANIRAY_WITHOUT_OLA)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE src/lintHelpers/NodeArrayOutputOLA.cpp)
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

target_include_directories(${CMAKE_PROJECT_NAME}
                           PRIVATE lib/fast-cpp-csv-parser)

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
