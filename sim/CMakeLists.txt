# Sim library — the simulation orchestrator and world model.
# Mirrors the structure of core/CMakeLists.txt.

add_library(sim src/environment/World.cpp)

target_include_directories(sim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(sim PUBLIC core)

# Tests
if(BUILD_TESTING)
    add_subdirectory(test)
endif()
