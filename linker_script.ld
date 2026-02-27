/* This script is specifically designed for the STM32F072CBT6 MCU. */
ENTRY(reset_handler)

/* Source: https://www.st.com/resource/en/reference_manual/rm0091-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf */
MEMORY
{
    FLASH (rx): ORIGIN = 0x08000000, LENGTH = 128K
    SRAM (rwx): ORIGIN = 0x20000000, LENGTH = 16K
    USB_SRAM (rw): ORIGIN = 0x40006000, LENGTH = 1K
}
_estack = ORIGIN(SRAM) + LENGTH(SRAM);

/* Source: https://www.st.com/resource/en/programming_manual/pm0215-stm32f0-series-cortexm0-programming-manual-stmicroelectronics.pdf */
SECTIONS {
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    .text :
    {
        . = ALIGN(4);

        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)

        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .ARM.extab (READONLY) :
    {
        . = ALIGN(4);

        *(.ARM.extab* .gnu.linkonce.armextab.*)

        . = ALIGN(4);
    } > FLASH

    .ARM (READONLY) :
    {
        . = ALIGN(4);

        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;

        . = ALIGN(4);
    } > FLASH

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;

        *(.data)
        *(.data*)

        . = ALIGN(4);
        _edata = .;
    } > SRAM AT > FLASH

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;

        *(.bss)
        *(.bss*)

        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > SRAM

    .usb_sram (NOLOAD) :
    {
        _susb_sram = .;
        *(.usb_sram)
        *(.usb_sram*)
        _eusb_sram = .;
    } > USB_SRAM
}
