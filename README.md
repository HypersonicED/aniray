aniray
======
A library for animating and orchestrating large arrays of emitters, actuators, displays, and more.

## Features

- Automatic subsampling node grid generation and caching
- Load node physical data from CSV
- Visualization framework independent (can be used with Cinder, oFx, etc.)
- Open Lighting Architecture (OLA) for lighting output allowing for robust many universe outputs of E1.31 (sACN), Artnet, KiNET, USB DMX adapters, and more.
- Extensible output classes for other output methods

## Requirements

- C++17
- clang or gcc, untested on other platforms
- Boost >= 1.74.0 (Boost Software License)
- OpenSSL >= 1.1.0 (OpenSSL License)
- libola >= 0.10.8 (LGPL) or turn off with `-DANIRAY_WITHOUT_OLA=ON` and bring your own output
- Bundled requirements (Git submodules):
  - [dmilos/color](https://github.com/dmilos/color) (Apache-2.0 License)
  - [ben-strasser/fast-cpp-csv-parser](https://github.com/ben-strasser/fast-cpp-csv-parser) (BSD-3-Clause License)

## Build and Lint Requirements

- CMake
- `iwyu` (include-what-you-use) with `-DANIRAY_WITH_LINT_=ON`
- `clang-tidy` with `-DANIRAY_WITH_LINT_=ON`

## Using

1. Install required dependencies above
2. Clone with submodules
   ```bash
   git clone --recursive https://github.com/HypersonicED/aniray.git
   ```
3. Either
   - Add to your `CMakeLists.txt`
   - Build the static library
     ```bash
     cmake -H. -B_builds/Release -DCMAKE_BUILD_TYPE=Release
     ```
   - Build the shared library
     ```bash
     cmake -H. -B_builds/Release -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
     ```
