/*
 * uart.h
 *
 * Created: 02.08.2016 4:05:49
 *  Author: LV
 */ 


#ifndef UART_H_
#define UART_H_

#include <avr/io.h>

//void init_UART(void);
void init_UART(unsigned long baudrate);
void send_Uart(unsigned char c);		//	Отправка байта
void send_Uart_str(unsigned char *s);	//	Отправка строки
void send_int_Uart(unsigned int c);		//	Отправка числа от 0000 до 9999
unsigned char getch_Uart(void);			//	Получение байта

#endif /* UART_H_ */