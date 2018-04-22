/* LED.h
*/

#include "LED.h"
#include "util/delay.h"


void LED_init(){
    LED_DDR |= 1<<LED_PIN;
}

void LED_HexBlink(uint8_t n) {
    uint8_t z = (n & 0xF0)>>4;
    LED_nBlink(z);
    _delay_ms(250);
    z = n & 0x0F;
    LED_nBlink(z);
    _delay_ms(250);
}
/*--------------------------------------------*/

void LED_nBlink(uint8_t n) {
    if (!n) {
        LED_On();
        _delay_ms(50);
        LED_Off();
        _delay_ms(50);
        LED_On();
        _delay_ms(50);
        LED_Off();
        _delay_ms(50);
    }
	else {
            for (uint8_t i = 0; i < n; i++)	LEDS_Blink_Once();
	}
}
/*--------------------------------------------*/

void LEDS_Blink_Once(void) {
        LED_On();
        _delay_ms(200);
        LED_Off();
        _delay_ms(200);
}
/*--------------------------------------------*/

void LEDS_Blink_Once_Short(void) {
        LED_On();
        _delay_ms(75);
        LED_Off();
        _delay_ms(75);
}
/*--------------------------------------------*/

void LED_On(void)
{
    LED_PORT |= 1<<LED_PIN;
}
/*--------------------------------------------*/

void LED_Off(void)
{
    LED_PORT &= ~(1<<LED_PIN);
}
/*--------------------------------------------*/
