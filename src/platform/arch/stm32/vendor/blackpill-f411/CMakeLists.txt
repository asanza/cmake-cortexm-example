# CMSIS library
set(TARGET cmsis_f411xe)

set(CMSIS_PATH ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS)

add_library(${TARGET} INTERFACE)

target_include_directories(${TARGET} INTERFACE 
    ${CMSIS_PATH}/Core/Include
    ${CMSIS_PATH}/Device/ST/STM32F4xx/Include
)


# Hal stm Library
set(TARGET vendor_stm32f411xE)

set(STM32_HAL ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F4xx_HAL_Driver)

add_library(${TARGET} STATIC
    ${STM32_HAL}/Src/stm32f4xx_hal_gpio.c
    ${STM32_HAL}/Src/stm32f4xx_hal_rcc.c
    ${STM32_HAL}/Src/stm32f4xx_hal_cortex.c
    ${STM32_HAL}/Src/stm32f4xx_hal.c
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/system_stm32f4xx.c
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/main.c
)
target_compile_options(${TARGET} PRIVATE -Wno-unused-function)
target_compile_definitions(${TARGET} PUBLIC STM32F411xE)

target_include_directories(${TARGET} PUBLIC 
    ${STM32_HAL}/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
)

target_link_libraries(${TARGET} PUBLIC cmsis_f411xe)