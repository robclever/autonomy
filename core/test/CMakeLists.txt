# Core library tests

add_executable(test_frames src/test_frames.cpp)
target_link_libraries(test_frames PRIVATE core)

add_executable(test_clock src/test_clock.cpp)
target_link_libraries(test_clock PRIVATE core)

# Register with CTest
include(CTest)
add_test(NAME test_frames COMMAND test_frames)
add_test(NAME test_clock COMMAND test_clock)
