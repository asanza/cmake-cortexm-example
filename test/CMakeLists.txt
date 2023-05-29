add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/support/unity)

macro(ADD_UNIT_TEST TEST_NAME SOURCES)
set(TARGET test_example)
    add_executable(${TARGET} 
        ${SOURCES}
        ${CMAKE_CURRENT_LIST_DIR}/support/crt/startup.c
    )

    target_link_options(${TARGET} PRIVATE
        -T ${CMAKE_CURRENT_LIST_DIR}/support/crt/linker_script.ld
        -Wl,--gc-sections
        -Wl,--warn-common
        -Wl,--check-sections
        -Wl,--print-memory-usage
        -Wl,--whole-archive
        -Wl,--no-whole-archive
        -Wl,--Map=${TARGET}.map
        --specs=rdimon.specs
        -lrdimon
        -lc
    )
    target_link_libraries(${TARGET} PRIVATE unity)
    add_test(NAME ${TARGET} COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/support/qemu-test-fixture.py ${TARGET})
endmacro()

add_unit_test(test_example ${CMAKE_CURRENT_LIST_DIR}/test_example.c)