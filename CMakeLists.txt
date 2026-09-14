cmake_minimum_required(VERSION 3.10)

project(autonomy VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build tests by default
option(BUILD_TESTING "Build tests" ON)

# Enable CTest
enable_testing()

# Add core library
add_subdirectory(core)

# Add environment library (ground-truth world model)
add_subdirectory(environment)

# Add systems library (sensors, fusion, autonomy)
add_subdirectory(systems)

# Add sim library
add_subdirectory(sim)

