# Environment library — ground-truth world model (Landmark + World) and
# observation generation. Structured body-frame observations from a pose.

add_library(environment src/World.cpp)

target_include_directories(environment
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(environment PUBLIC core)

# Tests
if(BUILD_TESTING)
    add_subdirectory(test)
endif()
