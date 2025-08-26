/*
 * assembly.s
 *
 */
 
 @ DO NOT EDIT
	.syntax unified
    .text
    .global ASM_Main
    .thumb_func

@ DO NOT EDIT
vectors:
	.word 0x20002000
	.word ASM_Main + 1

@ DO NOT EDIT label ASM_Main
ASM_Main:

	@ Some code is given below for you to start with
	LDR R0, RCC_BASE  		@ Enable clock for GPIOA and B by setting bit 17 and 18 in RCC_AHBENR
	LDR R1, [R0, #0x14]
	LDR R2, AHBENR_GPIOAB	@ AHBENR_GPIOAB is defined under LITERALS at the end of the code
	ORRS R1, R1, R2
	STR R1, [R0, #0x14]

	LDR R0, GPIOA_BASE		@ Enable pull-up resistors for pushbuttons
	MOVS R1, #0b01010101
	STR R1, [R0, #0x0C]
	LDR R1, GPIOB_BASE  	@ Set pins connected to LEDs to outputs
	LDR R2, MODER_OUTPUT
	STR R2, [R1, #0]
	MOVS R2, #0         	@ NOTE: R2 will be dedicated to holding the value on the LEDs

@ TODO: Add code, labels and logic for button checks and LED patterns

main_loop:
    @ Run Default Pattern
    STR R2, [R1, #0x14]      @ Offset 0x14 is ODR - write R2 to LEDs
                             @ Display current pattern (in R2) on LEDs (GPIOB_ODR)

    @ Check which delay to use based on current mode
    @ R7 will be our temporary delay counter holder
    LDR R7, =delay_mode      @ Load address of delay mode flag
    LDR R8, [R7]             @ Load delay mode value (0 = long, 1 = short)
    CMP R8, #1
    BNE use_long_delay

use_short_delay:
    LDR R7, SHORT_DELAY_CNT  @ Load short delay value
    B delay_start

use_long_delay:
    LDR R7, LONG_DELAY_CNT   @ Load long delay value

delay_start:
    @ Delay loop
delay_loop:
    SUBS R7, R7, #1          @ Decrement delay counter
    BNE delay_loop           @ Loop until counter reaches 0

    @ Increment the pattern based on current increment mode
    @ R6 will be our temporary increment value holder
    LDR R6, =increment_mode  @ Load address of increment mode
    LDR R8, [R6]             @ Load increment value (1 or 2)
    ADD R2, R2, R8           @ Add increment to pattern

    @ --- PART 2: Check Buttons ---
    LDR R0, GPIOA_BASE
    LDR R3, [R0, #0x10]      @ Read GPIOA_IDR into R3 (buttons state)

    @ Check SW0 and SW1 together first (bits 0 and 1)
    MOVS R4, #0b00000011     @ Mask for SW0 and SW1
    ANDS R5, R3, R4          @ Isolate SW0 and SW1 bits
    CMP R5, #0b00000001      @ Is only SW0 pressed? (SW0=0, SW1=1)
    BEQ mode_two
    CMP R5, #0b00000010      @ Is only SW1 pressed? (SW0=1, SW1=0)
    BEQ mode_three
    CMP R5, #0b00000000      @ Are both SW0 and SW1 pressed? (SW0=0, SW1=0)
    BEQ mode_four

    @ Check SW2 (bit 2)
    MOVS R4, #0b00000100     @ Mask for SW2
    TST R3, R4
    BEQ mode_five            @ If pressed (bit=0), branch

    @ Check SW3 (bit 3)
    MOVS R4, #0b00001000     @ Mask for SW3
    TST R3, R4
    BEQ mode_freeze          @ If pressed (bit=0), branch

    @ Default Mode 1: No buttons pressed
    B mode_one

mode_one:
    @ Set increment to 1, delay to long
    LDR R6, =increment_mode
    MOV R8, #1
    STR R8, [R6]
    LDR R7, =delay_mode
    MOV R8, #0
    STR R8, [R7]
    B main_loop

mode_two:
    @ Set increment to 2, delay to long
    LDR R6, =increment_mode
    MOV R8, #2
    STR R8, [R6]
    LDR R7, =delay_mode
    MOV R8, #0
    STR R8, [R7]
    B main_loop

mode_three:
    @ Set increment to 1, delay to short
    LDR R6, =increment_mode
    MOV R8, #1
    STR R8, [R6]
    LDR R7, =delay_mode
    MOV R8, #1
    STR R8, [R7]
    B main_loop

mode_four:
    @ Set increment to 2, delay to short
    LDR R6, =increment_mode
    MOV R8, #2
    STR R8, [R6]
    LDR R7, =delay_mode
    MOV R8, #1
    STR R8, [R7]
    B main_loop

mode_five:
    @ Force pattern to 0xAA and display it
    MOV R2, #0xAA
    STR R2, [R1, #0x14]     @ Update LEDs immediately

    @ Wait in loop until SW2 is released
mode_five_loop:
    LDR R3, [R0, #0x10]     @ Read GPIOA_IDR again
    MOVS R4, #0b00000100    @ Mask for SW2
    TST R3, R4
    BEQ mode_five_loop      @ Loop if button is still pressed

    B main_loop             @ Return to main loop when released

mode_freeze:
    @ Keep displaying the current pattern while frozen
    STR R2, [R1, #0x14]     @ Keep LEDs on

    @ Wait in loop until SW3 is released
mode_freeze_loop:
    LDR R3, [R0, #0x10]     @ Read GPIOA_IDR again
    MOVS R4, #0b00001000    @ Mask for SW3
    TST R3, R4
    BEQ mode_freeze_loop    @ Loop if button is still pressed

    B main_loop             @ Return to main loop when released
@ Data section for mode variables
.data
.align
increment_mode: .word 1     @ Start with increment by 1
delay_mode:     .word 0     @ Start with long delay


write_leds:
	STR R2, [R1, #0x14]
	B main_loop

@ LITERALS; DO NOT EDIT
	.align
RCC_BASE: 			.word 0x40021000
AHBENR_GPIOAB: 		.word 0b1100000000000000000
GPIOA_BASE:  		.word 0x48000000
GPIOB_BASE:  		.word 0x48000400
MODER_OUTPUT: 		.word 0x5555

@ TODO: Add your own values for these delays
LONG_DELAY_CNT: 	.word 1400000
SHORT_DELAY_CNT: 	.word 600000
