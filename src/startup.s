.cpu cortex-m0
.thumb

.section .text
.global reset_handler
.global default_handler
.global main
.global isr_vector

.extern _etext
.extern _sidata
.extern _sdata
.extern _edata
.extern _sbss
.extern _ebss
.extern _susb_sram
.extern _eusb_sram

.section .isr_vector, "a", %progbits
.type isr_vector, %object
isr_vector:
    .word _estack
    .word reset_handler
    .word nmi_handler
    .word hard_fault_handler
    .word 0x00000000
    .word 0x00000000
    .word 0x00000000
    .word 0x00000000
    .word 0x00000000
    .word 0x00000000
    .word 0x00000000
    .word svcall_handler
    .word 0x00000000
    .word 0x00000000
    .word pendsv_handler
    .word systick_handler
    .word wwdg_handler
    .word pvd_vddio2_handler
    .word rtc_handler
    .word flash_handler
    .word rcc_crs_handler
    .word exti0_1_handler
    .word exti2_3_handler
    .word exti4_15_handler
    .word tsc_handler
    .word dma_ch1_handler
    .word dma_ch2_3_dma2_ch1_2_handler
    .word dma_ch4_5_6_7_dma2_ch3_4_5_handler
    .word adc_comp_handler
    .word tim1_brk_up_trg_com_handler
    .word tim1_cc_handler
    .word tim2_handler
    .word tim3_handler
    .word tim6_dac_handler
    .word tim7_handler
    .word tim14_handler
    .word tim15_handler
    .word tim16_handler
    .word tim17_handler
    .word i2c1_handler
    .word i2c2_handler
    .word spi1_handler
    .word spi2_handler
    .word usart1_handler
    .word usart2_handler
    .word usart3_4_5_6_7_8_handler
    .word cec_can_handler
    .word usb_handler

.weak nmi_handler
.weak hard_fault_handler
.weak svcall_handler
.weak pendsv_handler
.weak systick_handler
.weak wwdg_handler
.weak pvd_vddio2_handler
.weak rtc_handler
.weak flash_handler
.weak rcc_crs_handler
.weak exti0_1_handler
.weak exti2_3_handler
.weak exti4_15_handler
.weak tsc_handler
.weak dma_ch1_handler
.weak dma_ch2_3_dma2_ch1_2_handler
.weak dma_ch4_5_6_7_dma2_ch3_4_5_handler
.weak adc_comp_handler
.weak tim1_brk_up_trg_com_handler
.weak tim1_cc_handler
.weak tim2_handler
.weak tim3_handler
.weak tim6_dac_handler
.weak tim7_handler
.weak tim14_handler
.weak tim15_handler
.weak tim16_handler
.weak tim17_handler
.weak i2c1_handler
.weak i2c2_handler
.weak spi1_handler
.weak spi2_handler
.weak usart1_handler
.weak usart2_handler
.weak usart3_4_5_6_7_8_handler
.weak cec_can_handler
.weak usb_handler

.thumb_set nmi_handler, default_handler
.thumb_set hard_fault_handler, default_handler
.thumb_set svcall_handler, SVC_Handler                          @ Handled by FreeRTOS
.thumb_set pendsv_handler, PendSV_Handler                       @ Handled by FreeRTOS
.thumb_set systick_handler, SysTick_Handler                     @ Handled by FreeRTOS
.thumb_set wwdg_handler, default_handler
.thumb_set pvd_vddio2_handler, default_handler
.thumb_set rtc_handler, default_handler
.thumb_set flash_handler, default_handler
.thumb_set rcc_crs_handler, default_handler
.thumb_set exti0_1_handler, default_handler
.thumb_set exti2_3_handler, default_handler
.thumb_set exti4_15_handler, default_handler
.thumb_set tsc_handler, default_handler
.thumb_set dma_ch1_handler, default_handler
.thumb_set dma_ch2_3_dma2_ch1_2_handler, default_handler
.thumb_set dma_ch4_5_6_7_dma2_ch3_4_5_handler, default_handler
.thumb_set adc_comp_handler, default_handler
.thumb_set tim1_brk_up_trg_com_handler, default_handler
.thumb_set tim1_cc_handler, default_handler
.thumb_set tim2_handler, default_handler
.thumb_set tim3_handler, default_handler
.thumb_set tim6_dac_handler, default_handler
.thumb_set tim7_handler, default_handler
.thumb_set tim14_handler, default_handler
.thumb_set tim15_handler, default_handler
.thumb_set tim16_handler, default_handler
.thumb_set tim17_handler, default_handler
.thumb_set i2c1_handler, default_handler
.thumb_set i2c2_handler, default_handler
.thumb_set spi1_handler, default_handler
.thumb_set spi2_handler, default_handler
.thumb_set usart1_handler, default_handler
.thumb_set usart2_handler, default_handler
.thumb_set usart3_4_5_6_7_8_handler, default_handler
.thumb_set cec_can_handler, default_handler
.thumb_set usb_handler, default_handler

.section .text
.type default_handler, %function
default_handler:
    b reset_handler

.type copy_data, %function
copy_data:
    ldr R0, =_sidata    @ R0 stores the address of the current byte in flash.
    ldr R1, =_sdata     @ R1 stores the address of the current byte in SRAM.

    ldr R2, =_edata
    ldr R3, =_sdata
    sub R2, R2, R3      @ R2 stores the size of data.

    ldr R3, =0          @ R3 stores the current index.
copy_data_loop:
    cmp R3, R2
    bge copy_data_loop_end

    mov R4, #0
    ldrb R4, [R0]        @ R4 contains the current byte.
    strb R4, [R1]

    add R0, R0, #1
    add R1, R1, #1
    add R3, R3, #1
    b copy_data_loop
copy_data_loop_end:
    bx LR

.type zero_out_bss, %function
zero_out_bss:
    ldr R0, =_sbss      @ R0 stores the current bss byte.
    ldr R1, =_ebss      @ R1 stores the end of the bss section.
zero_out_bss_loop:
    cmp R0, R1
    bge zero_out_bss_loop_end

    mov R2, #0
    strb R2, [R0]

    add R0, R0, #1
    b zero_out_bss_loop
zero_out_bss_loop_end:
    bx LR

.type reset_handler, %function
reset_handler:
    bl copy_data
    bl zero_out_bss
    bl main
