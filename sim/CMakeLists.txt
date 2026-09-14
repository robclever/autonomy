# Sim library — the simulation orchestrator and world model.
# Mirrors the structure of core/CMakeLists.txt.

add_library(sim
    src/environment/CameraDevice.cpp
    src/environment/RadarDevice.cpp
    src/scenarios/MeasurementAdapter.cpp
    src/scenarios/Scenario.cpp
    src/scenarios/SensorSuite.cpp
    src/scenarios/Trajectory.cpp
    src/scenarios/TrajectoryBuilder.cpp
)

target_include_directories(sim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(sim PUBLIC core systems environment)

# Tests
if(BUILD_TESTING)
    add_subdirectory(test)
endif()
