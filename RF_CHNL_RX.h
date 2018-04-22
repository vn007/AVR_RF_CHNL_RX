/*
 * RF_CHNL.h
 *
 * Created: 19.11.2016 10:08:01
 *  Author: LV
 */


#ifndef RF_CHNL_H_
#define RF_CHNL_H_

#include <util/delay.h>

#define F_CPU 16000000UL

#define SS_PORT		&PORTB
#define SS_PIN		PB2
#define CE_PORT		&PORTB
#define CE_PIN		PB1
#define SCK_PORT	&PORTB
#define SCK_PIN		PB5
#define MOSI_PORT	&PORTB
#define MOSI_PIN	PB3
#define MISO_PORT	&PORTB
#define MISO_PIN	PB4

#define RF_CHANNEL_NO 73
#define Primary_RX 1

#define AddrP0_B0 0xA0
#define AddrP0_B1 0xA1
#define AddrP0_B2 0xA2
#define AddrP0_B3 0xA3
#define AddrP0_B4 0xA4

#define AddrP1_B0 0xB0
#define AddrP1_B1 0xB1
#define AddrP1_B2 0xB2
#define AddrP1_B3 0xB3
#define AddrP1_B4 0x11

#define AddrP2_B0 0x12
#define AddrP3_B0 0x13
#define AddrP4_B0 0x14
#define AddrP5_B0 0x15

#define STOP while(1);

void process_message(char *message);
volatile bool rf_interrupt;

#endif /* RF_CHNL_H_ */
