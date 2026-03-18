.cpu cortex-m0
.thumb

.section .text
.global delay_us_at_8mhz

@ Implements void delay_us_at_8mhz(uint32_t us).
@ The argument value must be at least 2.
.type delay_us_at_8mhz, %function
delay_us_at_8mhz:
    @ blx was used to call this function, that is 3 cycles.
    @ Compensate time wasted calling this function and returning from it.
    sub R0, R0, #1 @ 1 cycle.
    nop @ 1 cycle.
delay_us_at_8mhz_loop:
    nop @ 4 cycles.
    nop
    nop
    nop
    sub R0, R0, #1 @ 1 cycle.
    bne delay_us_at_8mhz_loop @ 3 (1) cycles.
    nop @ 2 cycles.
    nop
    bx LR @ 3 cycles.
