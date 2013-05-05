/**
 * Atmega Steuerung für Bierdreher
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

#define PHASE_A     (PINC & 1<<PC0)
#define PHASE_B     (PINC & 1<<PC1)

#define XTAL 4000000

//KTY-Offset
#define OFFSET 116

#include "lcd.h"

/************************************************************************/
/* Globals                                                              */
/************************************************************************/

//importierter Drehimpulscode
volatile int8_t enc_delta;          // -128 ... 127
static int8_t last;

//ADC
volatile uint16_t adc_val;

/************************************************************************/
/* Subroutines                                                          */
/************************************************************************/

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

/************************************************************************/
/* Interrupts                                                           */
/************************************************************************/

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

ISR( ADC_vect ){
	adc_val = ADCW;
	ADCSRA |= (1<<ADSC);
}


/************************************************************************/
/* Main                                                                 */
/************************************************************************/

int main(void){



	/************************************************************************/
	/* Init PWM                                                             */
	/************************************************************************/
	
	//Prescaling auf 0
	TCCR2 &= ~((1<<FOC2) | (1<<WGM21) | (1<<COM20) | (1<<CS22) | (1<<CS21));
	TCCR2 |= ((1<<WGM20) | (1<<COM21) |  (1<<CS20));


	/************************************************************************/
	/* Init rotary pulse encoder                                            */
	/************************************************************************/
	
	DDRB |= (1<<DDB3);
	DDRB |= (1<<DDB4);
	
	//Timer0, mit Interrupt ca. jede 1000/s
	TCCR0 |= ((1<<CS01));
	TIMSK |= (1<<TOIE0);
	int ocr_set = 255;
	encode_init();
	
	/************************************************************************/
	/* LCD Initialization                                                   */
	/************************************************************************/
	
	lcd_init(LCD_DISP_ON_CURSOR_BLINK);
	lcd_puts("Here for beer");
	
	/************************************************************************/
	/* ADC_init                                                             */
	/************************************************************************/
	
	//Referenzspannung auf 2,56V
	ADMUX |= ((1<<REFS1)|(1<<REFS0));
	
	//Inoutquelle wählen, ADC3
	ADMUX &= ~((1<<MUX3) | (1<<MUX2));
	ADMUX |= ((1<<MUX1) | (1<<MUX0));
	
	//ADC Prescaler für 4MHz auf 64
	ADCSRA |= ((1<<ADPS2) | (1<<ADPS1));
	ADCSRA &= ~((1<<ADPS0));
	
	//ADC-Mode auf SingleConversion und Interrupt aktivieren
	ADCSRA |= ((1<<ADEN) | (1<<ADIE) | (1<<ADSC));
	
	/************************************************************************/
	/* Init help variables                                                  */
	/************************************************************************/
	
	uint16_t help = 0;
	char str[10];
	//uint16_t help_i = 0;
	int offset = 0;

	double temp;
	
	/************************************************************************/
	/* Start                                                                */
	/************************************************************************/
	sei();
	while(1){
		
		//Read gray-code and set PWM
		offset = encode_read4();
		ocr_set -= offset;
		OCR2 = ocr_set;	

	//Das sollte auch in nen Timer!
	if(help == 32000){
		lcd_gotoxy(0,1);
		temp = ((double)adc_val * (2.5/7.7125)) - (double)OFFSET;
		//dtostrf(temp, 4, 0, str);
		sprintf(str, "%3.0f", temp);
		//sprintf(str, "%3d", adc_val);
		lcd_puts(strcat(str, "C / "));
		//itoa(255-OCR2, str, 10);
		sprintf(str, "%3d", 255-OCR2);
		lcd_puts(str);
		//lcd_gotoxy(1,5);
		help = 0;
	}					
	help++;
	
	}

}