/*
 * Macro to divide a 4-byte value by a 1-byte value yielding a 4-byte
 * quotient and 2-byte remainder.
 *
 * On entry:   r25..r22 contains the 4-byte dividend
 *             r20 contains the 1-byte divisor
 *
 * On exit:    r25..r22  contains the 4-byte quotient
 *             r18 contains the 1-byte remainder
 *
    CP R2,R4 ; compare lower bytes
    CPC R1,R3 ; compare upper bytes

If the carry flag is set now, R1:R2 is smaller than R3:R4.
 */

.text
.global div32x8
.type div32x8, @function

div32x8:
	push 	r17
	push 	r18
;	push 	r19

    clr     r18
 ;   clr     r19
    ldi     r17, 32

udi0:
    lsl     r22			;multiply r25..r22 by 2
    rol     r23
    rol     r24
    rol     r25

    rol     r18			;...putting the result into r19:r18:r25..r22
 ;   rol     r19
    brcs    udi1		;goto udi1 if r19 was 0b1xxxxxxx

    cp      r18, r20	
 ;   cpc     r19, r21	
    brcs    udi2		;goto udi2 if r19:r18 is smaller than r21:r20
udi1:
    sub     r18, r20
 ;   sbc     r19, r21	;subtract r18 = r18 - r20
    ori     r22, 1		;OR r22 with 0x01
udi2:
    dec     r17
    brne    udi0

;	pop 	r19
	pop 	r18
	pop 	r17
    ret 

.size div32x8, .-div32x8
