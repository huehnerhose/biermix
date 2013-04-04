/**
 * Atmega Steuerung für Bierdreher
 */
#include <avr/io.h>

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

	TCCR2 &= ((1<<FOC2) | (1<<WGM21) | (1<<COM20));
	TCCR2 |= ((1<<WGM20) | (1<<COM21) | (1<<CS22) | (1<<CS21) |(1<<CS20));

	OCR2 = 128;

	while(1){
	}

}
