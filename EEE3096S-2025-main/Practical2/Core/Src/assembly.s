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
    @ Read button states from GPIOA_IDR (offset 0x10)
    LDR R0, GPIOA_BASE      @ Load GPIOA base address into R0
    LDR R3, [R0, #0x10]    @ Read GPIOA_IDR into R3 
    @0x10 is the offset to the Input Data Register (IDR)

    @ Check if SW2 is pressed
    MOVS R4, #0b00000100    @ Mask for SW2 
    TST R3, R4
    BEQ SW2_pressed         @ If pressed, branch to SW2_pressed    

    @ Check if SW3 is pressed
    MOVS R4, #0b00001000    @ Mask for SW3
    TST R3, R4
    BEQ sw3_pressed         @ If pressed, branch to SW3_pressed

    @ If neither SW2 nor SW3 pressed, go to default pattern
    B default_pattern

SW2_pressed:
    MOVS R2, #0xAA          @ Set pattern to 0xAA (10101010)
    STR R2, [R1, #0x14]     @ Update LEDs immediately. Store In Register Format = STR Rt, [Rn, #offset]
    @0x14 is the exact memory offset from the GPIO base address to reach the Output Data Register (ODR)

    @ if SW2 is still pressed-> stay in loop
SW2_loop:
    LDR R3, [R0, #0x10]     @ Read GPIOA_IDR again
    MOVS R4, #0b00000100    @ Mask for SW2
    TST R3, R4
    BEQ SW2_loop            @ Loop if button is still pressed

    B delay_selection     @ When released, continue with delay

sw3_pressed:
    STR R2, [R1, #0x14]     @ Keep displaying current pattern

    @ if SW3 is still pressed-> stay in loop
sw3_loop:
    LDR R3, [R0, #0x10]     @ Read GPIOA_IDR again
    MOVS R4, #0b00001000    @ Mask for SW3
    TST R3, R4
    BEQ sw3_loop            @ Loop if button is still pressed

    B delay_selection     @ When released, continue with delay

default_pattern:
    @ Check if SW0 is pressed or not
    MOVS R4, #0b00000001    @ Mask for SW0 (bit 0)
    TST R3, R4
    BNE increment_by_one    @ If not pressed, increment by 1

increment_by_two:
    ADDS R2, R2, #2         @ SW0 pressed: increment by 2
    B delay_selection

increment_by_one:
    ADDS R2, R2, #1         @ SW0 not pressed: increment by 1

delay_selection:
    @ Determine delay time based on SW1
    MOVS R4, #0b00000010    @ Mask for SW1 (bit 1)
    TST R3, R4
    BNE seven_delay      @ If SW1 not pressed, use long delay(0.7s)

three_delay: @else use short delay(0.3s)
    LDR R7, SHORT_DELAY_CNT @ Load short delay value (0.3s)
    B pattern_display

seven_delay:
    LDR R7, LONG_DELAY_CNT  @ Load long delay value (0.7s)

pattern_display:
    @ Display the current pattern on LEDs
    STR R2, [R1, #0x14]     @ Write to GPIOB_ODR

    @ Execute delay loop
delay_loop:
    SUBS R7, R7, #1         @ Decrement delay counter
    BNE delay_loop          @ Loop until counter reaches 0

    @ Loop forever
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
