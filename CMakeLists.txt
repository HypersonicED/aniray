cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(ledpixel LANGUAGES CXX)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
    YES
    CACHE BOOL "Export all symbols")

option(LEDPIXEL_WITHOUT_OLA "Build without Open Lighting Architecture" NO)

add_definitions(-DBOOST_LOG_DYN_LINK)
find_package(Boost 1.74.0 REQUIRED COMPONENTS log)
find_package(OpenSSL 1.1.0 REQUIRED)
# find_package(Catch2 REQUIRED)

find_program(IWYU_EXE NAMES include-what-you-use iwyu REQUIRED)
find_program(CLANG_TIDY_EXE NAMES "clang-tidy" "clang-tidy-13" "clang-tidy-12"
                                  "clang-tidy-11" REQUIRED)
# add_subdirectory(lib/Format.cmake)

if(NOT LEDPIXEL_WITHOUT_OLA)
  find_package(PkgConfig)
  pkg_check_modules(LIBOLA libola>=0.10.8 REQUIRED)
endif()

add_library(
  ${CMAKE_PROJECT_NAME}
  include/ledpixel/DMXAddr.hpp
  include/ledpixel/LEDPixel.hpp
  include/ledpixel/LEDPixelAnimation.hpp
  include/ledpixel/LEDPixels.hpp
  include/ledpixel/LEDPixelsOutput.hpp
  include/ledpixel/LEDPixelsSampler.hpp
  src/DMXAddr.cpp
  src/LEDPixel.cpp
  src/lintHelper.cpp)
if(NOT LEDPIXEL_WITHOUT_OLA)
  # Private to avoid double linking https://stackoverflow.com/a/34533415
  target_sources(${CMAKE_PROJECT_NAME} PRIVATE include/ledpixel/LEDPixelsOutputOLA.hpp)
endif()
target_compile_features(${CMAKE_PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Werror)
set_target_properties(
  ${CMAKE_PROJECT_NAME}
  PROPERTIES LINKER_LANGUAGE CXX
             # CXX_EXTENSIONS OFF
            #  CXX_INCLUDE_WHAT_YOU_USE "${IWYU_EXE}" "-Xiwyu" "--cxx17ns"
             CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")

target_include_directories(
  ${CMAKE_PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_include_directories(${CMAKE_PROJECT_NAME}
                           PRIVATE lib/fast-cpp-csv-parser)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE lib/color/src)

install(
  TARGETS ${CMAKE_PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)



# add_executable(tests test/main.cpp test/test_DMXAddr.cpp )
# set_target_properties( tests PROPERTIES LINKER_LANGUAGE CXX CXX_EXTENSIONS
# OFF) # CXX_CLANG_TIDY
# "${CLANG_TIDY_EXE};-header-filter=.*;-checks=*;-warnings-as-errors=*")
# target_include_directories(tests PUBLIC test/) target_link_libraries(tests
# ${Cath2_LIBRARIES} ${CMAKE_PROJECT_NAME} ) include(Catch)
# catch_discover_tests(tests) install( TARGETS tests RUNTIME DESTINATION bin )
