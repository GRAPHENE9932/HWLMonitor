cmake_minimum_required(VERSION 3.21)
include(FetchContent)

project("HWLMonitor" C ASM)

set(CPU cortex-m0)
add_definitions(-DSTM32F042x6)
add_definitions(-DUSE_FULL_LL_DRIVER)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

set(LINKER_SCRIPT "${PROJECT_SOURCE_DIR}/linker_script.ld")

add_compile_options(
    -mcpu=${CPU}
    $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-Os>
    -Wall
    -Wundef
    -Wfatal-errors
    --param=min-pagesize=0
    -mthumb
    -ffunction-sections
    -fdata-sections
    -T ${LINKER_SCRIPT}
)

add_link_options(
    -mthumb
    -Wl,--gc-sections
    -T ${LINKER_SCRIPT}
)

add_executable(
    hwlmonitor

    src/startup.s
    src/main.c
    src/usb_hid.c
    src/i2c.c
    src/sh1106.c
    src/drawing_task.c
    src/primitive_drawing.c
    src/gui/mode_menu.c
    submodules/stm32f0xx-hal-driver/Src/stm32f0xx_ll_i2c.c
    submodules/stm32f0xx-hal-driver/Src/stm32f0xx_ll_gpio.c
)

target_include_directories(
    hwlmonitor PUBLIC
    include/
    ${CMAKE_CURRENT_BINARY_DIR} # Needed for generated header(s).
    submodules/CMSIS_6/CMSIS/Core/Include
    submodules/cmsis-device-f0/Include
    submodules/stm32f0xx-hal-driver/Inc
    submodules/FreeRTOS-Kernel/portable/GCC/ARM_CM0
    submodules/FreeRTOS-Kernel/include
)

set_property(TARGET hwlmonitor PROPERTY C_STANDARD 23)

# Compile and link FreeRTOS.
set(FREERTOS_PORT GCC_ARM_CM0)
set(FREERTOS_HEAP 1)
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE include)
add_subdirectory(submodules/FreeRTOS-Kernel)

target_link_libraries(
    hwlmonitor PRIVATE
    freertos_kernel
)

# Download, extract and process a font.
set(FONT_TTF "${FETCHCONTENT_BASE_DIR}/font-src/ttf - Mx \\(mixed outline+bitmap\\)/Mx437_IBM_VGA_9x16.ttf")
set(FONT_H "${CMAKE_CURRENT_BINARY_DIR}/font.h")
FetchContent_Declare(
    font
    URL https://int10h.org/oldschool-pc-fonts/download/oldschool_pc_font_pack_v2.2_linux.zip
    URL_HASH MD5=e4e114151d7d9d4baf7b0f69b6645043
    DOWNLOAD_EXTRACT_TIMESTAMP true
)
FetchContent_MakeAvailable(
    font
)
add_custom_command(
    OUTPUT ${FONT_H}
    COMMAND python ${PROJECT_SOURCE_DIR}/scripts/ttf-to-header.py -i ${FONT_TTF} -o ${FONT_H}
)
add_custom_target(generate_font_h DEPENDS ${FONT_H})
add_dependencies(hwlmonitor generate_font_h)

set_target_properties(hwlmonitor PROPERTIES OUTPUT_NAME hwlmonitor.elf)
add_custom_target(
    upload
    COMMAND arm-none-eabi-objcopy -O binary hwlmonitor.elf firmware.bin
    COMMAND st-flash write firmware.bin 0x08000000
)
