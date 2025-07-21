cmake_minimum_required(VERSION 3.21)
include(ExternalProject)

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
    -Wno-mainv
    -Wundef
    -Wfatal-errors
    --param=min-pagesize=0
    -nostdlib
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
)

target_include_directories(
    hwlmonitor PUBLIC
    include/
    submodules/CMSIS_6/CMSIS/Core/Include
    submodules/cmsis-device-f0/Include
    submodules/FreeRTOS-Kernel/portable/GCC/ARM_CM0
    submodules/FreeRTOS-Kernel/include
)

set_property(TARGET hwlmonitor PROPERTY C_STANDARD 23)

set(FREERTOS_PORT GCC_ARM_CM0)
set(FREERTOS_HEAP 1)
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE include)
add_subdirectory(submodules/FreeRTOS-Kernel)

target_link_libraries(
    hwlmonitor PRIVATE
    freertos_kernel
)

set_target_properties(hwlmonitor PROPERTIES OUTPUT_NAME hwlmonitor.elf)

add_custom_target(
    upload
    COMMAND arm-none-eabi-objcopy -O binary hwlmonitor.elf firmware.bin
    COMMAND st-flash write firmware.bin 0x08000000
)
