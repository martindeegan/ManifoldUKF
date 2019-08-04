cmake_minimum_required (VERSION 3.10)
project(manifold_ukf)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS ON)

# Set software source dir
set(SOFTWARE_SOURCE_DIR ${PROJECT_SOURCE_DIR})


# Set all compiled libraries to be placed in software/lib
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Set all compiled executables to be placed in software/bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


# Add path to search for modules
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake")

add_subdirectory(source)
