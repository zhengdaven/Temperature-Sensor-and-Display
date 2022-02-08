;***************************************************************************
;*
;* Title: temp_meas.asm
;* Author: Daven Zheng
;* Version:	1
;* Last updated: 11/17/21
;* Target: AVR128DB48
;*
;* DESCRIPTION
;* This program uses the ADC to measure an analog input voltage connected to
;* its AIN11 (PE3) pin. Then it scale the binary ADC result to convert it to
;* a binary representation of the temperature in degrees celcius. The binary
;* value in degrees celcius is then converted to BCD and displayed on the
;* seven-segment display
;* 
;* VERSION HISTORY
;* 1.0 Original version
;***************************************************************************
.equ PERIOD_EXAMPLE_VALUE = 50		;Put correct number in for 40.0Hz

.dseg
led_display: .byte 4				;allocate 4 byte for variable led_display
bcd_entries: .byte 4				;allocate 4 byte for variable bcd_entries
digit_num: .byte 1					;allocate 1 byte for variable digit_num

.cseg
	jmp start						;reset vector executed a power ON

.org TCA0_OVF_vect
	jmp toggle_pin_ISR

.org ADC0_RESRDY_vect
	jmp ADC_convert					;vector for all PORTE pin change IRQs

start:
	ldi r16, 0x00					;load r16 with all 0's
	sts PORTC_DIR, r16				;configure PORTC as inputs
	ldi XH, HIGH(PORTC_PIN0CTRL)	;X points to PORTC_PIN0CTRL
	ldi XL, low(PORTC_PIN0CTRL)
	ldi r17, 8						;loop control variable

pullups:
	ld r16, X						;load value of PORTC_PINnCTRL
	ori r16, 0x88					;enable pullups and invert input
	st X+, r16						;store result
	dec r17							;decrement r17
	brne pullups					;repeat loop until all pullups are enabled

start1:
	ldi r16, 0xFF					;load r16 with all 1's
	sts PORTA_DIR, r16				;configure PORTA as outputs
	sts PORTD_DIR, r16				;configure PORTD as outputs
	ldi r16, 0x00					;load r16 with all 0's
	sts PORTE_DIR, r16				;configure PORTE as inputs

	ldi r16, TCA_SINGLE_WGMODE_NORMAL_gc	;WGMODE normal
	sts TCA0_SINGLE_CTRLB, r16

	ldi r16, TCA_SINGLE_OVF_bm				;enable overflow interrupt
	sts TCA0_SINGLE_INTCTRL, r16

	ldi r16, LOW(PERIOD_EXAMPLE_VALUE)		;set the period
	sts TCA0_SINGLE_PER, r16
	ldi r16, HIGH(PERIOD_EXAMPLE_VALUE)
	sts TCA0_SINGLE_PER + 1, r16	;load period low byte then high byte

	ldi r16, TCA_SINGLE_CLKSEL_DIV256_gc | TCA_SINGLE_ENABLE_bm
	sts TCA0_SINGLE_CTRLA, r16				;set clock and start timer
	ldi ZH, HIGH(bcd_entries)		;Z points to bcd_entries
	ldi ZL, LOW(bcd_entries)		;Z points to bcd_entries
	ldi YH, HIGH(led_display)		;Y points to led_display
	ldi YL, LOW(led_display)		;Y points to led_display
	ldi r23, 0x00					;load r20 with all 0's

	ldi r16, 0x03					;load r16 with 0x03
	sts VREF_ADC0REF, r16			;configure the internal VREF for 2.5V
	ldi r16, 0x04					;load r16 with 0x04
	sts PORTE_PIN3CTRL, r16			;configure PE3 as an analog input
	ldi r16, 0x0A					;load r16 with 0x0A
	sts ADC0_CTRLC, r16			;select internal VREF and ADC0 clock prescalar
	ldi r16, 0x01					;load r16 with 0x01
	sts ADC0_CTRLA, r16				;enable ADC0
	ldi r16, 0x0B					;load r16 with 0x0B
	sts ADC0_MUXPOS, r16			;select ADC0 multiplexer input
	ldi r16, 0x01					;load r16 with 0x01
	sts ADC0_COMMAND, r16			;start a conversion
	sts ADC0_INTCTRL, r16
	call post_display				;test the functionality of display

	sei								;enable global interrupts

main:
	nop
	rjmp main

;***************************************************************************
;*
;* "multiplex_display" - Multiplex the Four Digit LED Display
;*
;* Description:
;* Updates a single digit of the display and increments the
;* digit_num to the digit position to be displayed next.
;*
;* Author: Daven Zheng
;* Version: 1
;* Last updated: 11/02/21
;* Target: AVR128DB48
;* Number of words: 41
;* Number of cycles: 73
;* Low registers modified: none
;* High registers modified: none
;*
;* Parameters:
;* led_display: a four byte array that holds the segment values
;* for each digit of the display, led_display[0] holds the segment pattern
;* for digit 0 (the rightmost digit) and so on.
;* digit_num: byte variable, the least significant two bits are the index of
;* the last digit displayed
;*
;* Returns: Outputs segment pattern and turns on digit driver for the next
;* position in the display to be turned ON
;*
;* Notes: The segments are controlled by PORTD (dp, a through g), the digit
;* drivers are controlled by PORTA (PA7 - PA4, digit 0 - 3).
;*
;***************************************************************************
multiplex_display:
	ldi ZH, HIGH(bcd_entries)		;Z points to bcd_entries
	ldi ZL, LOW(bcd_entries)		;Z points to bcd_entries
	ldi YH, HIGH(led_display)		;Y points to led_display
	ldi YL, LOW(led_display)		;Y points to led_display
	push r17					;pushing all the registers used in this
	push r18					;routine to the stack
	push r19

	sts digit_num, r23			;store digit_num with r20
	lds r17, digit_num			;load digit_num into r17
	cpi r17, 0x00				;compare r17 with 0x00
	breq display0				;branch to display0 if equal
	cpi r17, 0x01				;compare r17 with 0x01
	breq display1				;branch to display1 if equal
	cpi r17, 0x02				;compare r17 with 0x02
	breq display2				;branch to display2 if equal
	cpi r17, 0x03				;compare r17 with 0x03
	breq display3				;branch to display3 if equal

display0:
	ldd r18, Y + 0				;load the value at address Y into r18
	ori r18, 0x80
	ldi r19, 0xF0				;load r19 with 0xF0
	sts PORTA_OUT, r19			;turn off display
	sts PORTD_OUT, r18			;store r18 to PORTD_OUT
	ldi r19, 0xE0				;load r19 with 1110 0000
	sts PORTA_OUT, r19			;turn on the last digit on the display
	inc r23						;increment r20
	rjmp return1

display1:
	ldd r18, Y + 1				;load the value at address Y + 1 into r18
	ldi r19, 0xF0				;load r19 with 0xF0
	sts PORTA_OUT, r19			;turn off display
	sts PORTD_OUT, r18			;store r18 to PORTD_OUT
	ldi r19, 0xD0				;load r19 with 1101 0000
	sts PORTA_OUT, r19			;turn on the last digit on the display
	inc r23						;increment r20
	rjmp return1

display2:
	ldd r18, Y + 2				;load the value at address Y + 2 into r18
	ori r18, 0x80
	ldi r19, 0xF0				;load r19 with 0xF0
	sts PORTA_OUT, r19			;turn off display
	sts PORTD_OUT, r18			;store r18 to PORTD_OUT
	ldi r19, 0xB0				;load r19 with 1011 0000
	sts PORTA_OUT, r19			;turn on the last digit on the display
	inc r23						;increment r20
	rjmp return1

display3:
	ldd r18, Y + 3				;load the value at address Y + 3 into r18
	ori r18, 0x80
	ldi r19, 0xF0				;load r19 with 0xF0
	sts PORTA_OUT, r19			;turn off display
	sts PORTD_OUT, r18			;store r18 to PORTD_OUT
	ldi r19, 0x70				;load r19 with 0111 0000
	sts PORTA_OUT, r19			;turn on the last digit on the display
	ldi r23, 0x00				;load r20 with all 0's
	rjmp return1

return1:
						;popping all the registers used in this
	pop r19						;routine from the stack
	pop r18
	pop r17
	ret

;***************************************************************************
;* 
;* "hex_to_7seg" - Hexadecimal to Seven Segment Conversion
;*
;* Description: Converts a right justified hexadecimal digit to the seven
;* segment pattern required to display it. Pattern is right justified a
;* through g. Pattern uses 0s to turn segments on ON.
;*
;* Author:			Ken Short
;* Version:			0.1						
;* Last updated:		101221
;* Target:			AVR128DB48
;* Number of words: 16
;* Number of cycles: 16
;* Low registers modified: none
;* High registers modified: none
;*
;* Parameters: r17: hex digit to be converted
;* Returns: r17: seven segment pattern. 0 turns segment ON
;*
;* Notes: 
;*
;***************************************************************************
hex_to_7seg:
	push ZL						;store Z in stack
	push ZH
    ldi ZH, HIGH(hextable * 2)  ;set Z to point to start of table
    ldi ZL, LOW(hextable * 2)
    ldi r16, $00                ;add offset to Z pointer
	andi r17, 0x0F				;mask for low nibble
    add ZL, r17
    adc ZH, r16
    lpm r17, Z                  ;load byte from table pointed to by z
	pop ZH						;load Z from stack
	pop ZL
	ret

    ;Table of segment values to display digits 0 - 9
    ;!!! seven values must be added
hextable: .db $01, $4F, $12, $06, $4C, $24, $20, $0F, $00, $04, $08, $60, $31, $42, $30, $38

;***************************************************************************
;* 
;* "toggle_pin_ISR" - interupt delay
;*
;* Description: global interrupt for multiplex display
;*
;* Author: Daven Zheng
;* Version: 1
;* Last updated: 11/09/21
;* Target: AVR128DB48
;* Number of words: 12
;* Number of cycles: 25
;* Low registers modified: none
;* High registers modified: none
;*
;* Parameters: 
;*
;* Returns: call multipex_display after interupt delay
;*
;* Notes: 
;*
;***************************************************************************
toggle_pin_ISR:
	push r16			;save register
	in r16, CPU_SREG
	push r16
	push r17

	call multiplex_display

	ldi r16, TCA_SINGLE_OVF_bm	;clear OVF flag
	sts TCA0_SINGLE_INTFLAGS, r16

	pop r17				;restore registers
	pop r16
	out CPU_SREG, r16
	pop r16

	reti

;***************************************************************************
;* 
;* "post_display" - 1 sec display
;*
;* Description: turns on all segments and all digits on the display for 1
;* second
;*
;* Author: Daven Zheng
;* Version: 1
;* Last updated: 10/26/21
;* Target: AVR128DB48
;* Number of words: 6
;* Number of cycles: 12
;* Low registers modified: none
;* High registers modified: none
;*
;* Parameters: 
;*
;* Returns: turn on all segments and digit for the display for 1 second
;*
;* Notes: 
;*
;***************************************************************************
post_display:
	ldi r16, 0x00
	std Y + 0, r16
	std Y + 1, r16
	std Y + 2, r16
	std Y + 3, r16
	call mux_digit_delay
	ldi r16, 0xFF
	std Y + 0, r16
	std Y + 1, r16
	std Y + 2, r16
	std Y + 3, r16
	ret

;***************************************************************************
;* 
;* "mux_digit_delay" - 1s delay 
;*
;* Description: lets the system run through a finite loop that takes 1s
;* to complete
;*
;* Author: Daven Zheng
;* Version: 1
;* Last updated: 10/26/21
;* Target: AVR128DB48
;* Number of words: 6
;* Number of cycles: 13271591
;* Low registers modified: none
;* High registers modified: none
;*
;* Parameters: 
;*
;* Returns: System delay
;*
;* Notes: 
;*
;***************************************************************************
mux_digit_delay:			;function to delay
	ldi r23, 40
outer_loop1:
	ldi r21, 255				;delay duration
outer_loop:
	ldi r22, 133			;delay duration
inner_loop:
	dec r22					;decrease delay duration
	brne inner_loop
	dec r21					;decrease delay duration
	brne outer_loop
	dec r23
	brne outer_loop1
	ret

;***************************************************************************
;* 
;* "ADC_convert" - interupt for ADC conversion to store the temperature on
;* to be displayed later
;*
;* Description: 
;* to complete
;*
;* Author: Daven Zheng
;* Version: 1
;* Last updated: 10/26/21
;* Target: AVR128DB48
;* Number of words: 6
;* Number of cycles: 13271591
;* Low registers modified: none
;* High registers modified: none
;*
;* Parameters: 
;*
;* Returns: store the temperature to be later displayed.
;*
;* Notes: 
;*
;***************************************************************************
ADC_convert:
	push r16
	push r17
	push r18
	push r19
	push r20
	push r21
	push r22
	lds r16, ADC0_RESL				;read value at ADC0_RESL
	lds r17, ADC0_RESH				;read value at ADC0_RESL
	ldi r19, HIGH(2500)					;load upper high byte of 2500 to r19
	ldi r18, LOW(2500)					;load low byte of 2500 to r18
	call mpy16u						;multiply RES with 2500
	//outputs are 21-18
loop:								;loop for division
 	ldi r22, 4						;load r22 with 4
loop1:
	lsr r21							;shift r21 to the right
	ror r20							;rotate r20 to the right
	ror r19							;rotate r19 to the right
	dec r22							;decrement r22
	brne loop1						;repeat loop1 until r22 = 0
	mov r16, r19					;copy r19 into r16
	mov r17, r20					;copy r20 into r17
	call bin2BCD16					;convert the low and high byte to bcd
	mov r16, r14					;copy high byte of bcd to r16
	andi r16, 0x0F					;mask the lower nibble of r16
	cpi r16, 0x05					;compare digit 2 to 5
	brlo lower500					;branch to lower500 if lower than 5

greater500:
	mov r16, r14					;copy high byte of bcd to r16
	ldi r17, 0x05					;load r17 with 0x05
	call BCDsub						;subtract r16 and r17
	andi r16, 0x0F					;mask r16 for lower nibble
	ldi ZH, HIGH(bcd_entries)		;Z points to bcd_entries
	ldi ZL, LOW(bcd_entries)		;Z points to bcd_entries
	std Z + 2, r16					;store lower nibble in Z + 2
	ldi r16, 0x00					;load r16 with all 0's
	std Z + 3, r16					;store 0 into Z + 3
	mov r16, r13					;copy r13 into r16
	andi r16, 0xF0					;mask for upper nibble
	swap r16						;swap upper and lower nibble
	mov r17, r13					;copy r13 to r17
	andi r17, 0x0F					;mask r13 for lower nibble
	std Z + 1, r16					;store r16 in Z + 1
	std Z + 0, r17					;store r17 in Z + 0
	ldd r17, Z + 0			;load value at address Z to r17
	call hex_to_7seg		;convert r17 to hex pattern on 7 seg display
	std Y + 0, r17			;store r17 to address Y

	ldd r17, Z + 1			;load value at address Z to r17
	call hex_to_7seg		;convert r17 to hex pattern on 7 seg display
	std Y + 1, r17			;store r17 to address Y

	ldd r17, Z + 2			;load value at address Z to r17
	call hex_to_7seg		;convert r17 to hex pattern on 7 seg display
	std Y + 2, r17			;store r17 to address Y

	ldd r17, Z + 3			;load value at address Z to r17
	call hex_to_7seg		;convert r17 to hex pattern on 7 seg display
	std Y + 3, r17			;store r17 to address Y
	ldi r16, 0x01			;load r16 with 0x01
	sts ADC0_INTFLAGS, r16
	sts ADC0_COMMAND, r16	;reset command flag

	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	reti

lower500:
	ldi ZH, HIGH(bcd_entries)		;Z points to bcd_entries
	ldi ZL, LOW(bcd_entries)		;Z points to bcd_entries
	ldi r16, 0x99			;load r16 with 0x99
	mov r17, r13			;copy lower byte of bcd to r17
	call BCDsub				;subtract r16 and r17
	ldi r17, 0x01			;load r17 with 0x01
	add r16, r17			;add 1 to r16
	mov r17, r16			;copy r16 to r17
	andi r17, 0xF0			;mask r17 for upper nibble
	swap r17				;swap upper and lower nibble
	andi r16, 0x0F			;mask r16 for lower nibble
	std Z + 1, r17			;store r17 in Z + 1
	std Z + 0, r16			;store r16 in Z + 0
	ldi r16, 0x04			;load r16 with 0x04
	mov r17, r14			;copy higher byte of bcd to r17
	andi r17, 0x0F			;mask r17 for lower nibble
	call BCDsub				;subtract r16 and r17
	andi r16, 0x0F			;mask r16 for lower nibble
	std Z + 2, r16			;store r16 in Z + 2
	ldd r17, Z + 0			;load value at address Z to r17
	call hex_to_7seg		;convert r17 to hex pattern on 7 seg display
	std Y + 0, r17			;store r17 to address Y

	ldd r17, Z + 1			;load value at address Z to r17
	call hex_to_7seg		;convert r17 to hex pattern on 7 seg display
	std Y + 1, r17			;store r17 to address Y

	ldd r17, Z + 2			;load value at address Z to r17
	call hex_to_7seg		;convert r17 to hex pattern on 7 seg display
	std Y + 2, r17			;store r17 to address Y

	ldi r17, 0xFE			;load r17 to turn on segment g
	std Y + 3, r17			;store r17 to address Y
	ldi r16, 0x01			;load r16 with 0x01
	sts ADC0_INTFLAGS, r16
	sts ADC0_COMMAND, r16	;reset command flag
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	reti

;***************************************************************************
;*
;* "bin2BCD16" - 16-bit Binary to BCD conversion
;*
;* This subroutine converts a 16-bit number (fbinH:fbinL) to a 5-digit
;* packed BCD number represented by 3 bytes (tBCD2:tBCD1:tBCD0).
;* MSD of the 5-digit number is placed in the lowermost nibble of tBCD2.
;*
;* Number of words	:25
;* Number of cycles	:751/768 (Min/Max)
;* Low registers used	:3 (tBCD0,tBCD1,tBCD2)
;* High registers used  :4(fbinL,fbinH,cnt16a,tmp16a)	
;* Pointers used	:Z
;*
;***************************************************************************

;***** Subroutine Register Variables

.dseg
tBCD0: .byte 1  // BCD digits 1:0
tBCD1: .byte 1  // BCD digits 3:2
tBCD2: .byte 1  // BCD digits 4

.cseg
.def	tBCD0_reg = r13		;BCD value digits 1 and 0
.def	tBCD1_reg = r14		;BCD value digits 3 and 2
.def	tBCD2_reg = r15		;BCD value digit 4

.def	fbinL = r16		;binary value Low byte
.def	fbinH = r17		;binary value High byte

.def	cnt16a	=r18		;loop counter
.def	tmp16a	=r19		;temporary value

;***** Code

bin2BCD16:
    push fbinL
    push fbinH
    push cnt16a
    push tmp16a


	ldi	cnt16a, 16	;Init loop counter	
    ldi r20, 0x00
    sts tBCD0, r20 ;clear result (3 bytes)
    sts tBCD1, r20
    sts tBCD2, r20
bBCDx_1:
    // load values from memory
    lds tBCD0_reg, tBCD0
    lds tBCD1_reg, tBCD1
    lds tBCD2_reg, tBCD2

    lsl	fbinL		;shift input value
	rol	fbinH		;through all bytes
	rol	tBCD0_reg		;
	rol	tBCD1_reg
	rol	tBCD2_reg

    sts tBCD0, tBCD0_reg
    sts tBCD1, tBCD1_reg
    sts tBCD2, tBCD2_reg

	dec	cnt16a		;decrement loop counter
	brne bBCDx_2		;if counter not zero

    pop tmp16a
    pop cnt16a
    pop fbinH
    pop fbinL
ret			; return
    bBCDx_2:
    // Z Points tBCD2 + 1, MSB of BCD result + 1
    ldi ZL, LOW(tBCD2 + 1)
    ldi ZH, HIGH(tBCD2 + 1)
    bBCDx_3:
	    ld tmp16a, -Z	    ;get (Z) with pre-decrement
	    subi tmp16a, -$03	;add 0x03

	    sbrc tmp16a, 3      ;if bit 3 not clear
	    st Z, tmp16a	    ;store back

	    ld tmp16a, Z	;get (Z)
	    subi tmp16a, -$30	;add 0x30

	    sbrc tmp16a, 7	;if bit 7 not clear
        st Z, tmp16a	;	store back

	    cpi	ZL, LOW(tBCD0)	;done all three?
    brne bBCDx_3
        cpi	ZH, HIGH(tBCD0)	;done all three?
    brne bBCDx_3
rjmp bBCDx_1		

;***************************************************************************
;*
;* "mpy16u" - 16x16 Bit Unsigned Multiplication
;*
;* This subroutine multiplies the two 16-bit register variables 
;* mp16uH:mp16uL and mc16uH:mc16uL.
;* The result is placed in m16u3:m16u2:m16u1:m16u0.
;*  
;* Number of words	:14 + return
;* Number of cycles	:153 + return
;* Low registers used	:None
;* High registers used  :7 (mp16uL,mp16uH,mc16uL/m16u0,mc16uH/m16u1,m16u2,
;*                          m16u3,mcnt16u)	
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	mc16uL	=r16		;multiplicand low byte
.def	mc16uH	=r17		;multiplicand high byte
.def	mp16uL	=r18		;multiplier low byte
.def	mp16uH	=r19		;multiplier high byte
.def	m16u0	=r18		;result byte 0 (LSB)
.def	m16u1	=r19		;result byte 1
.def	m16u2	=r20		;result byte 2
.def	m16u3	=r21		;result byte 3 (MSB)
.def	mcnt16u	=r22		;loop counter

;***** Code

mpy16u:	clr	m16u3		;clear 2 highest bytes of result
	clr	m16u2
	ldi	mcnt16u,16	;init loop counter
	lsr	mp16uH
	ror	mp16uL

m16u_1:	brcc	noad8		;if bit 0 of multiplier set
	add	m16u2,mc16uL	;add multiplicand Low to byte 2 of res
	adc	m16u3,mc16uH	;add multiplicand high to byte 3 of res
noad8:	ror	m16u3		;shift right result byte 3
	ror	m16u2		;rotate right result byte 2
	ror	m16u1		;rotate result byte 1 and multiplier High
	ror	m16u0		;rotate result byte 0 and multiplier Low
	dec	mcnt16u		;decrement loop counter
	brne	m16u_1		;if not done, loop more
	ret

;***************************************************************************
;*
;* "BCDsub" - 2-digit packed BCD subtraction
;*
;* This subroutine subtracts the two unsigned 2-digit BCD numbers
;* "BCDa" and "BCDb" (BCDa - BCDb). The result is returned in "BCDa", and
;* the underflow carry in "BCDb".
;*
;* Number of words	:13
;* Number of cycles	:12/17 (Min/Max)
;* Low registers used	:None
;* High registers used  :2 (BCDa,BCDb)	
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	BCDa	=r16		;BCD input value #1
.def	BCDb	=r17		;BCD input value #2

;***** Code

BCDsub:
	sub	BCDa,BCDb	;subtract the numbers binary
	clr	BCDb
	brcc	sub_0		;if carry not clear
	ldi	BCDb,1		;    store carry in BCDB1, bit 0
sub_0:	brhc	sub_1		;if half carry not clear
	subi	BCDa,$06	;    LSD = LSD - 6
sub_1:	sbrs	BCDb,0		;if previous carry not set
	ret			;    return
	subi	BCDa,$60	;subtract 6 from MSD
	ldi	BCDb,1		;set underflow carry
	brcc	sub_2		;if carry not clear
	ldi	BCDb,1		;    clear underflow carry	
sub_2:	ret			