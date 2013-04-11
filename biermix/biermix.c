/**
 * Atmega Steuerung für Bierdreher
 * Umzug
 */
#include <avr/io.h>
#include <avr/interrupt.h>

#define PHASE_A     (PINC & 1<<PC0)
#define PHASE_B     (PINC & 1<<PC1)

#define XTAL 4000000


#include "lcd.h"
	//importierter Drehimpulscode
	volatile int8_t enc_delta;          // -128 ... 127
	static int8_t last;
	
	
void encode_init( void )
{
	int8_t new;
		
	new = 0;
	if( PHASE_A )
		new = 3;
	if( PHASE_B )
		new ^= 1;                   // convert gray to binary
	last = new;                   // power on state
	enc_delta = 0;
	
	//Timer, mit Interrupt ca. jede 1000/s
	TCCR0 |= ((1<<CS01));
	TIMSK |= (1<<TOIE0);
}
	
	
ISR( TIMER0_OVF_vect )             // 1ms for manual movement
{
	int8_t new, diff;
	new = 0;
	if( PHASE_A )
		new = 3;
	if( PHASE_B )
		new ^= 1;                   // convert gray to binary
	diff = last - new;                // difference last - new
	if( diff & 1 ){               // bit 0 = value (1)
		last = new;                 // store new as next last
		enc_delta += (diff & 2) - 1;        // bit 1 = direction (+/-)
	}
}


int8_t encode_read1( void )         // read single step encoders
{
	int8_t val;
	
	cli();
	val = enc_delta;
	enc_delta = 0;
	sei();
	return val;                   // counts since last call
}

int8_t encode_read4( void )         // read four step encoders
{
	int8_t val;
	
	cli();
	val = enc_delta;
	enc_delta = val & 3;
	sei();
	return val >> 2;
}

int main(void){
	/*
	 * configure timer 2 for phase correct PWM
	 * TCCR2:
	 * FOC2->0						compatibility
	 * WGM21->0, WGM20->1			phase correct PWM
	 * COM21->1, COM20->1/0			inverted/non-inverted
	 * CS22->1, CS21->1, CS20->1	Prescale @ 1024
	 *
	 * OCR2 -> Vergleichswert für Counter
	 *
	 * OC2 -> Outputpin = PB3, Pin17
	 * DDRB muss PB3 auf Output setzen
	 */

//	 x |= (1 << Bitnummer);  // Hiermit wird ein Bit in x gesetzt
//	 x &= ~(1 << Bitnummer); // Hiermit wird ein Bit in x geloescht
//	DDB --> 1 Ausgang, 0 Eingang

	//PB3 Als Ausgang definieren
	DDRB |= (1<<DDB3);
	DDRB |= (1<<DDB4);

	TCCR2 &= ((1<<FOC2) | (1<<WGM21) | (1<<COM20));
	TCCR2 |= ((1<<WGM20) | (1<<COM21) | (1<<CS22) | (1<<CS21) | (1<<CS20));

	//OCR2 = 0;
	
	

	int ocr_set = 0;
	encode_init();
	sei();
	
	lcd_init(LCD_DISP_ON_CURSOR_BLINK);
	lcd_puts("Foo");
	
	
	uint16_t help = 0;
	char str[16];
	int help_i = 0;
	int offset = 0;
	
	while(1){
		
		//Kontrolle gegen den Überlauf. 
		offset = encode_read4();
		if( !(ocr_set == 0 && offset < 0) || !(ocr_set == 255 && offset > 0) ){
			ocr_set += offset;
			OCR2 = ocr_set;	
		}
		
		//TODO: leg das auf nen Timer Call
		if(help == 655){
			lcd_gotoxy(0,1);
			help_i = OCR2;
			itoa(help_i, str, 10);
			lcd_puts(str);
			help = 0;
		}
		help++;
	}

}