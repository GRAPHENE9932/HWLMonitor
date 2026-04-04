cmake_minimum_required(VERSION 3.21)
include(FetchContent)

project("HWLMonitor" C ASM)

set(CPU cortex-m0)
add_definitions(-DSTM32F072xB)
add_definitions(-DUSE_FULL_LL_DRIVER)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

set(LINKER_SCRIPT "${PROJECT_SOURCE_DIR}/linker_script.ld")

set(FONT_TTF "${FETCHCONTENT_BASE_DIR}/font-src/ttf - Mx \\(mixed outline+bitmap\\)/Mx437_IBM_VGA_9x16.ttf")
set(FONT_H "${CMAKE_CURRENT_BINARY_DIR}/font.h")
set(IMAGES_FOLDER "${PROJECT_SOURCE_DIR}/images")
set(IMAGES_H "${CMAKE_CURRENT_BINARY_DIR}/images.h")

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
    -mcpu=${CPU}
    -mthumb
    -Wl,--gc-sections
    -T ${LINKER_SCRIPT}
)

add_executable(
    hwlmonitor

    src/startup.s
    src/main.c
    src/main_task.c
    src/utils.c
    src/usb_hid.c
    src/adc.c
    src/spi.c
    src/gpio.c
    src/st7735.c
    src/pwr_ctrl.c
    src/delay.s
    src/ui.c
    src/rcc.c
    src/rtc.c
    src/i2c.c
    src/opt4001.c
    src/gui/status_bar.c
    src/gui/menu.c
    src/gui/mode_statistics.c
    src/gui/gui.c
    src/gui/text.c
    $<$<CONFIG:Debug>:src/debug_utils.c>
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
    DEPENDS ${PROJECT_SOURCE_DIR}/scripts/ttf-to-header.py
)

add_custom_command(
    OUTPUT ${IMAGES_H}
    COMMAND python ${PROJECT_SOURCE_DIR}/scripts/pngs-to-header.py -i ${IMAGES_FOLDER} -o ${IMAGES_H}
    DEPENDS ${PROJECT_SOURCE_DIR}/scripts/pngs-to-header.py ${IMAGES_FOLDER}
)

add_custom_command(
    TARGET hwlmonitor
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary hwlmonitor.elf firmware.bin
    COMMAND arm-none-eabi-size hwlmonitor.elf
)

add_custom_target(generate_headers DEPENDS ${FONT_H} ${IMAGES_H})
add_dependencies(hwlmonitor generate_headers)

set_target_properties(hwlmonitor PROPERTIES OUTPUT_NAME hwlmonitor.elf)
add_custom_target(
    upload
    COMMAND st-flash --reset write firmware.bin 0x08000000
    DEPENDS hwlmonitor.elf
)
