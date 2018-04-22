#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>
#include "nrf24l01.h"
#include "nrf24l01-mnemonics.h"

#include "RF_CHNL_RX.h"
#include "uart.h"

void process_message(char *message);

volatile bool rf_interrupt = false;
nRF24L01 *rf;

int main(void) {

	init_UART(9600);
    _delay_ms(250);	// Также для nRF24 нужно 100ms для PowerOn Reset

//    uint8_t address[5] = { 0xA0, 0xA1, 0xA2, 0xA3, 0xA4 };
//    uint8_t address[5] = { DP0_B0, DP0_B1, DP0_B2, DP0_B3, DP0_B4 };


	nRF24L01 __rf = {
		{SS_PORT, SS_PIN},
		{CE_PORT, CE_PIN},
		{SCK_PORT, SCK_PIN},
		{MOSI_PORT, MOSI_PIN},
		{MISO_PORT, MISO_PIN},
		Primary_RX,
		0,
		RF_CHANNEL_NO,
		{AddrP0_B0, AddrP0_B1, AddrP0_B2, AddrP0_B3, AddrP0_B4},
		{AddrP1_B0, AddrP1_B1, AddrP1_B2, AddrP1_B3, AddrP1_B4},
		AddrP2_B0,
		AddrP3_B0,
		AddrP4_B0,
		AddrP5_B0
	};

	rf = &__rf;

	nRF24L01_init(rf);

    nRF24L01_activate_pipe(rf, 0);     // pipe = 0

    sei();

    nRF24L01Message msg;

    while (true) {
        if (rf_interrupt) {
            rf_interrupt = false;
            while (nRF24L01_data_received(rf)) {    // nRF24L01_data_received возвращает TRUE если FIFO не пуст
                nRF24L01_read_received_data(rf, &msg);
//                send_Uart(0x77);
                process_message((char *)msg.data);
            }

        }
    }

    return 0;
}

void process_message(char *message) {
    uint8_t dbgNum = 0;

//	send_Uart(0x77);

    if (strcmp(message, "ON") == 0)
        dbgNum = 0x80;
    else if (strcmp(message, "OFF") == 0)
        dbgNum = 0x40;

	send_Uart(dbgNum);
}

// nRF24L01 interrupt
ISR(INT0_vect) {
    rf_interrupt = true;
}
