# Systems library — sensors, fusion, autonomy.
# Compiled (not header-only): sensor models and SLAM algorithms have .cpp
# implementations. Header-only components can be added as INTERFACE targets
# later if desired.

add_library(systems
    src/autonomy/EkfSlam.cpp
    src/autonomy/GraphSlam.cpp
    src/sensors/camera/CameraSensor.cpp
    src/sensors/radar/RadarSensor.cpp
)

target_include_directories(systems
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(systems PUBLIC core environment)

# Tests
if(BUILD_TESTING)
    add_subdirectory(test)
endif()
