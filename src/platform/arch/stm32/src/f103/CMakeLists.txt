set(TARGET hal_stm32f103xB)

add_library(${TARGET} STATIC
    ${CMAKE_CURRENT_LIST_DIR}/hal_system.c
    ${CMAKE_CURRENT_LIST_DIR}/hal_gpio.c
)

target_link_libraries(${TARGET} PUBLIC vendor_stm32f103xB hal)