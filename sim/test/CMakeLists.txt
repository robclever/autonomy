# Sim library tests — layout mirrors sim/include/<component>
# Every .cpp file in this tree is built as a standalone test executable.

file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS *.cpp)

foreach(test_source ${TEST_SOURCES})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(${test_name} ${test_source})
    target_link_libraries(${test_name} PRIVATE sim)
endforeach()

# Register with CTest
include(CTest)
foreach(test_source ${TEST_SOURCES})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
