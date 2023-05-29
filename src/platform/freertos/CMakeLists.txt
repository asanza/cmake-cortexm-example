set(TARGET freertos)
set(FREERTOS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOSv202212.01)

set(FREERTOS_CONFIG_FILE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/Config CACHE STRING "")
set(FREERTOS_PORT GCC_ARM_CM3 CACHE STRING "")
set(FREERTOS_HEAP "3" CACHE STRING "FreeRTOS heap model number. 1 .. 5. Or absolute path to custom heap source file")
add_subdirectory(${FREERTOS_SOURCE_DIR}/FreeRTOS/Source)

add_library(${TARGET} STATIC ${CMAKE_CURRENT_LIST_DIR}/System/sbrk.c)
target_link_libraries(${TARGET} PUBLIC freertos_kernel)
target_link_options(${TARGET} PUBLIC -Wl,--undefined=_sbrk)