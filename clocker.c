/*
for Breakout v1.0.1 and later
code revision 1.0.6: changed the way resync happens: no longer forces all jacks high immediately, and debounces
code revision 1.0.5: fixed the skip array in cv_skip.inc file to include 128 values.
*/
/*
	curadc PORT	Pin	LABEL	Cable#	Name
	0	PC5	28	J6	5	Shuffle
	1	PC0	23	CV1	1	Rotate
	2	PC1	24	CV2	2	Slip
	3	PC2	25	J3	11	Pulse Width
	4	PC3	26	J4	9	Skip
	5	PC4	27	J5	7	Resync

		PD4	6	J8	13	Mute
		PD5	11	J7	15	Faster
				+15V	3
				N/C	4
				GND	6
				GND	8
				GND	10
				bias	12	N/C on breakout, bias on main board
				GND	14
				GND	16
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdarg.h>
#include <ctype.h>

#include "cv_skip.inc"


/** SETTINGS **/
//MIN_PW 10 is about 1.28ms
#define MIN_PW 10
#define MIN_ADC_DRIFT 1
#define USER_INPUT_POLL_TIME 100

#define NO_FREERUN 0

/**MNEMONICS**/
#define CLKIN 0
#define INTERNAL 9

/** PINS **/

#define CLOCK_IN_pin PD2
#define CLOCK_IN_init DDRD &= ~(1<<CLOCK_IN_pin)
#define CLOCK_IN (PIND & (1<<CLOCK_IN_pin))

#define CLOCK_LED_pin PD3
#define CLOCK_LED_init DDRD |=(1<<CLOCK_LED_pin)
#define CLOCK_LED_PORT PORTD

#define DEBUG_pin PD1
#define DEBUG_init DDRD|=(1<<DEBUG_pin)
#define DEBUGHIGH PORTD |= (1<<DEBUG_pin)
#define DEBUGLOW PORTD &= ~(1<<DEBUG_pin)
#define DEBUGFLIP PORTD ^= (1<<DEBUG_pin)

#define OUT_PORT1 PORTB
#define OUT_DDR1 DDRB
#define OUT_MASK1 0b00111111
#define OUT_init1 OUT_DDR1 |= OUT_MASK1

#define OUT_PORT2 PORTD
#define OUT_DDR2 DDRD
#define OUT_MASK2 0b11000000
#define OUT_init2 OUT_DDR2 |= OUT_MASK2

#define BREAKOUT_MASK 0b111100
#define BREAKOUT_PULLUP PORTC
#define BREAKOUT_PIN PINC
#define BREAKOUT_DDR DDRC
#define BREAKOUT_init BREAKOUT_DDR &= ~(BREAKOUT_MASK)

#define ADC_DDR DDRC
#define ADC_PORT PORTC
#define ADC_mask 0b00111111
#define NUM_ADC 6

#define MUTE_SWITCH (!(PIND & (1<<PD4)))
#define FASTER_SWITCH (PIND & (1<<PD5))
#define SWITCHES_PULLUP PORTD |= ((1<<PD4) | (1<<PD5))

#define RESYNC (PINC & (1<<PC4))

/** MACROS **/

#define ALLON(p,x) p &= ~(x)
#define ALLOFF(p,x) p |= (x)
#define ON_NOMUTE(p,x) p &= ~(1<<(x))
#define ON(p,x) if(MUTE_SWITCH) p &= ~(1<<(x))
#define OFF(p,x) p |= (1<<(x))


/** TIMER **/
#define TMROFFSET 0

volatile uint32_t tmr[10];
volatile uint32_t clockin_irq_timestamp=0;



//SIGNAL (SIG_OVERFLOW0){
SIGNAL (TIMER0_OVF_vect){
	tmr[0]++;
	tmr[1]++;
	tmr[2]++;
	tmr[3]++;
	tmr[4]++;
	tmr[5]++;
	tmr[6]++;
	tmr[7]++;
	tmr[8]++;
	tmr[9]++;
	TCNT0=0; // Re-init timer
}

uint32_t gettmr(uint8_t i){
	uint32_t result;
	cli();
	result = tmr[i];
	sei();
	return result;
}

void reset_tmr(uint8_t i){
	cli();
	tmr[i]=0;
	sei();
}

void inittimer(void){
	//Fast PWM , TOP at 0xFF, OC0 disconnected, Prescale @ FCK/8
	//@ 16MHz, timer increments at 2MHz, overflows at 128us
	//
	TCCR0A=(1<<WGM01) | (1<<WGM00) | (0<<COM0A0) | (0<<COM0A1);
	TCCR0B= (0<<WGM02) | (0<<CS00) | (1<<CS01) | (0<<CS02);
	TCNT0=TMROFFSET;
	TIMSK0|=(1<<TOIE0); 					// Enable timer overflow interrupt

	tmr[CLKIN]=0;
	tmr[1]=0;
	tmr[2]=0;
	tmr[3]=0;
	tmr[4]=0;
	tmr[5]=0;
	tmr[6]=0;
	tmr[7]=0;
	tmr[8]=0;
	tmr[INTERNAL]=0;
	sei();
}

ISR (INT0_vect){
	if (CLOCK_IN){
		clockin_irq_timestamp=tmr[CLKIN];
		tmr[CLKIN]=0;
		tmr[INTERNAL]=0;
	}
}

void pcint_init(void){
	//interrupt on any change on INT0 (PD2/clockin)
	EICRA = (1<<ISC00) | (0<<ISC01);
	EIMSK = (1<<INT0);
}


uint8_t diff(uint8_t a, uint8_t b);
inline uint8_t diff(uint8_t a, uint8_t b){
	if (a>b) return (a-b);
	else return (b-a);
}

uint32_t div32x8(uint32_t a, uint8_t b); 

uint32_t div32_8(uint32_t a, uint8_t b);
inline uint32_t div32_8(uint32_t a, uint8_t b){
//takes 27-42us if a is 3,5,6,7
//takes <5us if a is 1,2,4,8

	if (b==1) return (a);
	else if (b==2) return (a>>1);
	else if (b==4) return (a>>2);
	else if (b==8) return (a>>3);
	else if (b==16) return (a>>4);
	else return div32x8(a,b);
}



uint32_t calc_pw(uint8_t pw_adc, uint32_t period){

	if (pw_adc<4) return(MIN_PW);
	else if (pw_adc<6) return(period>>5);
	else if (pw_adc<10) return(period>>4);
	else if (pw_adc<16) return((period>>4)+(period>>6));
	else if (pw_adc<22) return((period>>4)+(period>>5));
	else if (pw_adc<28) return((period>>4)+(period>>5)+(period>>6));
	else if (pw_adc<32) return(period>>3);
	else if (pw_adc<64) return((period>>3)+(period>>5));
	else if (pw_adc<80) return((period>>3)+(period>>4));
	else if (pw_adc<96) return((period>>3)+(period>>4)+(period>>5));
	else if (pw_adc<112) return(period>>2);
	else if (pw_adc<128) return((period>>2)+(period>>4));
	else if (pw_adc<142) return((period>>2)+(period>>3));
	else if (pw_adc<160) return((period>>2)+(period>>3)+(period>>4));
	else if (pw_adc<178) return(period>>1);		
	else if (pw_adc<180) return((period>>1) + (period>>4));
	else if (pw_adc<188) return((period>>1) + (period>>3));
	else if (pw_adc<196) return((period>>1) + (period>>2));
	else if (pw_adc<202) return((period*3)>>2);
	else if (pw_adc<206) return((period*7)>>3);
	else return((period*15)>>4);


}

/** MAIN **/

int main(void){
        
	uint16_t poll_user_input=0;

	uint8_t adch=0;
	char current_adc_channel=0;
	uint8_t rotate_adc=0;
	uint8_t skip_adc=0;
	uint8_t pw_adc=0;
	uint8_t shuffle_adc=0, slippage_adc=0;
	uint8_t old_shuffle_adc=127, old_slippage_adc=127, old_pw_adc=127, old_skip_adc=127;

	uint8_t rotation=0, old_rotation=255;
	uint8_t faster_switch_state=0,old_faster_switch_state=127;
	uint8_t d0=0,d1=0,d2=0,d3=0,d4=0,d5=0,d7=0; //multiply-by amount for each jack
	uint8_t dd2=0,dd3=0,dd4=0,dd5=0,dd7=0;

	uint32_t period=16800;
	uint32_t per0=0,per1=0,per2=0,per3=0,per4=0,per5=0,per7=0;

	uint32_t pw=8400;
	uint32_t pw0=8400,pw1=4200,pw2=2800,pw3=2200,pw4=1680,pw5=1400,pw7=1050; //pulse width amount for each jack

	uint8_t skip_pattern=0xFF;

	uint32_t slipamt2=0,slipamt3=0,slipamt4=0,slipamt5=0,slipamt7=0;

	uint8_t p2=0,p3=0,p4=0,p5=0,p6=0; //current pulse # in the sequence for each jack

	uint16_t temp_u16;
	char got_internal_clock=0;

	uint32_t now=0;

	uint16_t t16, resync_state=0;

	int32_t slip2=0,slip3=0,slip4=0,slip5=0,slip6=0;

	unsigned char s2=0,s3=0,s4=0,s5=0,s6=0;

	uint8_t slip_every=2,slip_howmany=0;
	uint8_t update_pulse_params=1;
	uint8_t min_pw=MIN_PW;
	

	inittimer();

	CLOCK_IN_init; 
	CLOCK_LED_init;
	DEBUG_init;
	OUT_init1;
	OUT_init2;
	BREAKOUT_init; 

	//init the ADC:
	ADC_DDR &= ~(ADC_mask); //adc input
	ADC_PORT &= ~(ADC_mask); //disable pullup
	ADCSRA = (1<<ADEN);	//Enable ADC
	ADMUX = (1<<ADLAR);	//Left-Adjust	
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	ADCSRA |= (1<<ADSC);//set the Start Conversion Flag in the ADC Status Register

	skip_adc=0;
	shuffle_adc=0;
	pw_adc=127;
	rotate_adc=0;	
	slippage_adc=127;

	pcint_init();

	while(1){


		if (++poll_user_input>USER_INPUT_POLL_TIME){
			poll_user_input=0;

			/** READ ADC **/
			while( !(ADCSRA & (1<<ADIF)) );
			ADCSRA |= (1<<ADIF);		// Clear the flag by sending a logical "1"
			adch=ADCH;

			if (current_adc_channel==4) {
				if (diff(skip_adc,adch)>MIN_ADC_DRIFT)
					skip_adc=adch;
				if (adch==0) {
					skip_adc=0;
					skip_pattern=0xFF;
				}
			}else if (current_adc_channel==0){
				if (diff(shuffle_adc,adch)>MIN_ADC_DRIFT) shuffle_adc=adch;
			}else if (current_adc_channel==3){
				if (diff(pw_adc,adch)>MIN_ADC_DRIFT) pw_adc=adch;
			}else if (current_adc_channel==1){
				if (diff(rotate_adc,adch)>MIN_ADC_DRIFT) rotate_adc=adch;	
			}else if (current_adc_channel==2){ 
				if (diff(slippage_adc,adch)>MIN_ADC_DRIFT) slippage_adc=adch;
			}

			ADCSRA |= (1<<ADSC);		//set the Start Conversion Flag in the ADC Status Register
		
			if (++current_adc_channel>=NUM_ADC)
				current_adc_channel=0;

			ADMUX = (1<<ADLAR) | current_adc_channel; 

			if (current_adc_channel==0){ //Update parameters after reading all of them

				rotation=rotate_adc>>5;
				
				if (FASTER_SWITCH)
					faster_switch_state=1;
				 else 
					faster_switch_state=0;
				
				if ((rotation!=old_rotation) || (faster_switch_state!=old_faster_switch_state)){

				/*dN is the multiply-by amount for jack N 1..8*/
					d0=rotation + 1;
					d1=((1+rotation) & 7) + 1;
					d2=((2+rotation) & 7) + 1;
					d3=((3+rotation) & 7) + 1;
					d4=((4+rotation) & 7) + 1;
					d5=((5+rotation) & 7) + 1;
					d7=((7+rotation) & 7) + 1;

					dd2=d2;
					dd3=d3;
					dd4=d4;
					dd5=d5;
					dd7=d7;
	
					if (FASTER_SWITCH){ //*4
						d0=d0<<2;	
						d1=d1<<2;	
						d2=d2<<2;
						d3=d3<<2;	
						d4=d4<<2;
						d5=d5<<2;
						d7=d7<<2;
					}

					update_pulse_params=1;
				}


				if ((slippage_adc != old_slippage_adc) || (pw_adc!=old_pw_adc))
					update_pulse_params=1;


				if (skip_adc!=old_skip_adc){
					if (skip_adc>128) skip_pattern=~skip[0xFF-skip_adc];
					else skip_pattern=skip[skip_adc];
				}

				if (shuffle_adc != old_shuffle_adc){
					temp_u16=shuffle_adc & 0x7F;	//chop to 0-127|0-127
					temp_u16=(temp_u16 * 5) >>7; //scale 0..127 to 0-4
					slip_every=temp_u16 + 2;	//shift to 2-6

					if (shuffle_adc<=127)
						slip_howmany=0;
					else
						slip_howmany=((shuffle_adc & 0b00011000)>>3) + 1; //1..4

					if (slip_howmany>=slip_every) slip_howmany=slip_every-1; //1..(slip_every-1)
				}

				
				old_rotation=rotation;
				old_faster_switch_state=faster_switch_state;
				old_pw_adc=pw_adc;
				old_shuffle_adc=shuffle_adc;
				old_skip_adc=skip_adc;
				old_slippage_adc=slippage_adc;
			}
		}

/*
		if (RESYNC){
			if (resync_up==0){
				resync_up=1;
*/				
		if (RESYNC) t16=0xe000; else t16=0xe001; //1110 0000 0000 000{0=jack high | 1=jack low}
		resync_state = (resync_state<<1) | t16;
		if ((resync_state) == 0xFF00) { //low 5 times then high 8 times
/**/
				p2=0;p3=0;p4=0;p5=0;p6=0;
				s2=0;s3=0;s4=0;s5=0;s6=0;
				slip2=slipamt2;
				slip3=slipamt3;
				slip4=slipamt4;
				slip5=slipamt5;
				slip6=slipamt7;

				//ON(OUT_PORT1,0);
				//ON(OUT_PORT1,1);
				//ON(OUT_PORT1,2);
				//ON(OUT_PORT1,3);
				//ON(OUT_PORT1,4);
				//ON(OUT_PORT1,5);
				//ON(OUT_PORT2,7);
				//ON(OUT_PORT2,6);
		}
/**/
/*
			}
		} else resync_up=0;
*/


		if (CLOCK_IN)
			ON_NOMUTE(CLOCK_LED_PORT,CLOCK_LED_pin);
		else
			OFF(CLOCK_LED_PORT,CLOCK_LED_pin);



		/* 
			Check to see if the clock input timer tmr[INTERNAL] has surpassed the period
			If so, 
		*/
	//	if (NO_FREERUN){
	//		got_internal_clock=0;
	//	} else {
			now=gettmr(INTERNAL);
			if (now>=period){
				got_internal_clock=1;
			}
	//	}

		if (got_internal_clock || (clockin_irq_timestamp && CLOCK_IN)){

			/*	Clock IN received:
				Turn all outputs on
				Reset their timers
				Reset their pN counters
			*/
			cli();

				if (!clockin_irq_timestamp){
					if (tmr[INTERNAL]>period)
						tmr[INTERNAL]=tmr[INTERNAL]-period;
				} else {
					period=clockin_irq_timestamp;
					clockin_irq_timestamp=0;
				}

				tmr[1]=0;
				tmr[2]=0;
				tmr[3]=0;
				tmr[4]=0;
				tmr[5]=0;
				tmr[6]=0;
				tmr[7]=0;
				tmr[8]=0;
			sei();

			ON(OUT_PORT1,0);
			ON(OUT_PORT1,1);
			ON(OUT_PORT1,2);
			ON(OUT_PORT1,3);
			ON(OUT_PORT1,4);
			ON(OUT_PORT1,5);
			ON(OUT_PORT2,7);
			ON(OUT_PORT2,6);

			got_internal_clock=0;
			p2=0;p3=0;p4=0;p5=0;p6=0;
			s2=0;s3=0;s4=0;s5=0;s6=0;

			update_pulse_params=2;

		}


		if (update_pulse_params){
			pw=calc_pw(pw_adc, period);

			pw0=div32_8(pw,d0);
			pw1=div32_8(pw,d1);
			pw2=div32_8(pw,d2);
			pw3=div32_8(pw,d3);
			pw4=div32_8(pw,d4);
			pw5=div32_8(pw,d5);
			pw7=div32_8(pw,d7);

			min_pw=MIN_PW+1;

			if ((per0>min_pw) && (pw0<MIN_PW)) pw0=MIN_PW;
			if ((per1>min_pw) && (pw1<MIN_PW)) pw1=MIN_PW;
			if ((per2>min_pw) && (pw2<MIN_PW)) pw2=MIN_PW;
			if ((per3>min_pw) && (pw3<MIN_PW)) pw3=MIN_PW;
			if ((per4>min_pw) && (pw4<MIN_PW)) pw4=MIN_PW;
			if ((per5>min_pw) && (pw5<MIN_PW)) pw5=MIN_PW;
			if ((per7>min_pw) && (pw7<MIN_PW)) pw7=MIN_PW;


			per0=div32_8(period,d0);
			per1=div32_8(period,d1);
			per2=div32_8(period,d2);
			per3=div32_8(period,d3);
			per4=div32_8(period,d4);
			per5=div32_8(period,d5);
			per7=div32_8(period,d7);

			if (slippage_adc<2){
				slipamt2=0;slipamt3=0;slipamt4=0;slipamt5=0;slipamt7=0;
			} else {
				slipamt2=(slippage_adc * (per2-(pw2+1))) >> 8;
				slipamt3=(slippage_adc * (per3-(pw3+1))) >> 8;
				slipamt4=(slippage_adc * (per4-(pw4+1))) >> 8;
				slipamt5=(slippage_adc * (per5-(pw5+1))) >> 8;
				slipamt7=(slippage_adc * (per7-(pw7+1))) >> 8;
			}
			
			if (update_pulse_params==2){ //clock in received
				slip2=slipamt2;
				slip3=slipamt3;
				slip4=slipamt4;
				slip5=slipamt5;
				slip6=slipamt7;
			}
			
			update_pulse_params=0;
		}


/*
	The period (on-time) for each jack is calculated by dividing the
	period variable by the rotation amount for the jack (d0-d7). 
	Once this amount of time has passed, we should considering turn the jack on...

	Unless of course the Skip CV (skip_adc) is telling us to drop this note out...
	To figure this out, we use a beat counter variable (p2-p6) to keep track of the
	current beat in the measure for each jack.
	E.g. The x4 jack has a 4-beat measure, so its beat counter (p4) counts 0 1 2 3 0 1 2 3...

	Using the Skip CV value (skip_adc) we lookup a beat pattern (skip_pattern) that tells which beats to play and 
	which ones to drop out. Then we see if the current beat is a 1 or a 0 in the skip_pattern

	(skip_pattern>>(8-rotationCV)) & (1<<pulse#)
*/



		//Jack 1: x1, no skip, no shuffle
		//use d0,per0 which defaults to x1
		now=gettmr(1);
		if (now>=per0) {
			reset_tmr(1);
			ON(OUT_PORT1,0);
		}

		//Jack 2: x2, no skip, no shuffle
		//use d1/per1, it defaults to x2
		now=gettmr(2);
		if (now>=per1) {
			reset_tmr(2);
			ON(OUT_PORT1,1);
		}

		//Jack 3: x3, skip/shuffle
		//use d2,per2,slipamt2, which defaults to x3
		now=gettmr(3);
		if (now>=(per2+slip2)) {
			reset_tmr(3);

			if ((skip_pattern>>(8-dd2)) & (1<<p2))
				ON(OUT_PORT1,2);

			if (++p2>=dd2){
				p2=0;
				s2=0;
				slip2=slipamt2;}
		
			if (s2==slip_howmany) slip2=-1*slipamt2;
			else slip2=0;

			if (++s2>=slip_every){
				s2=0;
				slip2=slipamt2;
			}
		}


		//Jack 4: x4 skip/shuffle
		//use d3/per3/slipamt3, which default to x4
		now=gettmr(4);
		if (now>=(per3+slip3)) {
			reset_tmr(4);

			if ((skip_pattern>>(8-dd3)) & (1<<p3))
				ON(OUT_PORT1,3);

			if (++p3>=dd3){
				p3=0;
				s3=0;
				slip3=slipamt3;}

			if (s3==slip_howmany) slip3=-1*slipamt3;
			else slip3=0;

			if (++s3>=slip_every){
				s3=0;
				slip3=slipamt3;
			}
		}
		//Jack 5: x5 skip/shuffle
		//o4/p4/s4/slip4 use dd4/per4/slipamt4 which default to x5
		now=gettmr(5);
		if (now>=(per4+slip4)) {
			reset_tmr(5);

			if ((skip_pattern>>(8-dd4)) & (1<<p4))
				ON(OUT_PORT1,4);

			if (++p4>=dd4){
				p4=0;
				s4=0;
				slip4=slipamt4;}

			if (s4==slip_howmany) slip4=-1*slipamt4;
			else slip4=0;

			if (++s4>=(slip_every)){
				s4=0;
				slip4=slipamt4;
			}
		}

		//Jack 6: x6 skip/shuffle
		//p5/s5/slip5 use dd5/per5/slipamt5 which default to x6
		now=gettmr(6);
		if (now>=(per5+slip5)) {
			reset_tmr(6);

			if ((skip_pattern>>(8-dd5)) & (1<<p5))
				ON(OUT_PORT1,5);

			if (++p5>=dd5){
				p5=0;
				s5=0;
				slip5=slipamt5;}

			if (s5==slip_howmany) slip5=-1*slipamt5;
			else slip5=0;

			if (++s5>=(slip_every)){
				s5=0;
				slip5=slipamt5;
			}
		}

		//Jack 7: x8 skip/shuffle
		//p6/s6/slip6 use dd7/per7/slipamt7 which default to x8
		now=gettmr(7);
		if (now>=(per7+slip6)) {
			reset_tmr(7);

			if ((skip_pattern>>(8-dd7)) & (1<<p6))
				ON(OUT_PORT2,7);

			if (++p6>=dd7){
				p6=0;
				s6=0;
				slip6=slipamt7;}

			if (s6==slip_howmany) slip6=-1*slipamt7;
			else slip6=0;

			if (++s6>=(slip_every)){
				s6=0;
				slip6=slipamt7;
			}

		}

		//Jack 8: x8 no skip/shuffle
		//use d7/per7 which defaults to x8
		now=gettmr(8);
		if (now>=per7) {
			reset_tmr(8);
			ON(OUT_PORT2,6);
		}


/* Turn jacks off whenever the pulsewidth has expired*/

		if (gettmr(1)>=pw0) {OFF(OUT_PORT1,0);}
		if (gettmr(2)>=pw1) {OFF(OUT_PORT1,1);}
		if (gettmr(3)>=pw2) {OFF(OUT_PORT1,2);}
		if (gettmr(4)>=pw3) {OFF(OUT_PORT1,3);}
		if (gettmr(5)>=pw4) {OFF(OUT_PORT1,4);}
		if (gettmr(6)>=pw5) {OFF(OUT_PORT1,5);}
		if (gettmr(7)>=pw7) {OFF(OUT_PORT2,7);}
		if (gettmr(8)>=pw7) {OFF(OUT_PORT2,6);}

	}	//endless loop


	return(1);
}
