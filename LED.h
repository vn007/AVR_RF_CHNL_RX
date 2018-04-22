#ifndef LED_H_INCLUDED
#define LED_H_INCLUDED

#define LED_PORT        PORTC
#define LED_DDR         DDRC
#define LED_PIN         2

#include "stdint.h"
#include "avr/io.h"


void LED_init();
void LED_HexBlink(uint8_t n);
void LED_nBlink(uint8_t N);
void LEDS_Blink_Once(void);
void LEDS_Blink_Once_Short(void);
void LED_On(void);
void LED_Off(void);

#endif // LED_H_INCLUDED
