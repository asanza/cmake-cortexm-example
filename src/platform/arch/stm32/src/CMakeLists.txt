set(TARGET hal_stm32f103xB)

add_library(${TARGET} STATIC
    ${CMAKE_CURRENT_LIST_DIR}/f103/hal_system.c
    ${CMAKE_CURRENT_LIST_DIR}/f103/hal_gpio.c
)

target_link_libraries(${TARGET} PUBLIC vendor_stm32f103xB hal)