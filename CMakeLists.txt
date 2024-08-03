# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(raytracer)

# Set the C++ standard to C++17 (or any other version you prefer)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -O3 -fopenmp")

# Specify the source file(s)
add_executable(raytracer main.cpp)

# Optionally, add debugging information
set(CMAKE_BUILD_TYPE Debug)