set(TARGET blinky)

set(LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/linker-script.ld)
set(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/startup.c
    ${CMAKE_CURRENT_LIST_DIR}/main.c
)

add_executable(${TARGET}.elf ${SOURCES})

set_target_properties(${TARGET}.elf PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

target_link_options(${TARGET}.elf PRIVATE 
    -nostartfiles
    -ffreestanding
    -specs=nano.specs
    -T${LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET}.map
    -Wl,--cref
    -Wl,--gc-sections
)

target_link_libraries(${TARGET}.elf PRIVATE freertos hal_stm32f103xB)


# a custom target to facilitate the flashing of the device with ninja flash
add_custom_target(flash COMMAND
        openocd -f interface/stlink.cfg -f target/stm32f1x.cfg 
        -c "program blinky.elf verify reset exit"
        DEPENDS blinky.elf
)