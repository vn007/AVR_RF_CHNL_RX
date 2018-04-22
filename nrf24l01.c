#include <stdlib.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "nrf24l01.h"
#include "nrf24l01-mnemonics.h"


static void copy_address(uint8_t *source, uint8_t *destination, uint8_t addr_len);
inline static void set_as_output(gpio_pin pin);
inline static void set_high(gpio_pin pin);
inline static void set_low(gpio_pin pin);
static void spi_init(nRF24L01 *rf);
static uint8_t spi_transfer(uint8_t data);


//nRF24L01 *nRF24L01_create(void) {
//    nRF24L01 *rf = malloc(sizeof(nRF24L01));
//    memset(rf, 0, sizeof(nRF24L01));
//    return rf;
//}

void nRF24L01_init(nRF24L01 *rf) {
    set_as_output(rf->ss);
    set_as_output(rf->ce);

    set_high(rf->ss);
    set_low(rf->ce);

    spi_init(rf);

    nRF24L01_send_command(rf, FLUSH_RX, NULL, 0);
    nRF24L01_send_command(rf, FLUSH_TX, NULL, 0);
    nRF24L01_clear_interrupts(rf);

    uint8_t data;
//    data = _BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP) | _BV(PRIM_RX);
//    data = _BV(EN_CRC) | _BV(CRCO) | _BV(PRIM_RX);
    data = _BV(EN_CRC) | _BV(CRCO);
    data |= (rf->primary_rx? _BV(PRIM_RX) : 0);
    nRF24L01_write_register(rf, CONFIG, &data, 1);

    //LV: set RF channel
	data = rf->rf_channel_no;
	nRF24L01_write_register(rf, RF_CH, &data, 1);

    _delay_us(130);

    // enable Auto Acknowledge on all pipes
    data = _BV(ENAA_P0) | _BV(ENAA_P1) | _BV(ENAA_P2)
         | _BV(ENAA_P3) | _BV(ENAA_P4) | _BV(ENAA_P5);  //Auto Acknowledge on all pipes
    nRF24L01_write_register(rf, EN_AA, &data, 1);

    // enable Dynamic Payload on al pipes
    data = _BV(DPL_P0) | _BV(DPL_P1) | _BV(DPL_P2)
         | _BV(DPL_P3) | _BV(DPL_P4) | _BV(DPL_P5);
    nRF24L01_write_register(rf, DYNPD, &data, 1);

    // enable Dynamic Payload (global)
    data = _BV(EN_DPL);
    nRF24L01_write_register(rf, FEATURE, &data, 1);

    // disable all rx addresses
    data = 0;
    nRF24L01_write_register(rf, EN_RXADDR, &data, 1);      //потом будут включаться по одному в функции nRF24L01_activate_pipe

	//Activating datapipe0
	nRF24L01_activate_pipe(rf, 0);

    //LV: Setup of Automatic Retransmission
    data = 0x0F;
    nRF24L01_write_register(rf, SETUP_RETR, &data, 1);     //15 Re-Transmit on fail of Auto Acknowledge

    nRF24L01_read_register(rf, CONFIG, &data, 1);   // POWER UP
    data |= _BV(PWR_UP);                            //
    nRF24L01_write_register(rf, CONFIG, &data, 1);  // do POWER UP
    _delay_us(4500);    //LV: Tpd2stby for worst case with Xtal, see table 16 in datasheet

	// interrupt on falling edge of INT0 (PD2)
    EICRA |= _BV(ISC01);
    EIMSK |= _BV(INT0);

}

void nRF24L01_activate_pipe(nRF24L01 *rf, int pipe) {
    uint8_t addr[5];
    uint8_t *addr_ptr;
    uint8_t addr_len;

//    addr_ptr = (pipe < 2? rf->p0_address + pipe: rf->p2_address + pipe - 2);
//    addr_ptr = (pipe == 0? rf->p0_address: (pipe == 1? rf->p1_address: rf->p2_address_b0 + (uint8_t *)pipe - 2));
    addr_ptr = (pipe < 2? rf->p0_address + 5*pipe: &rf->p2_address_b0 + pipe - 2);
    addr_len = (pipe < 2? 5: 1);

    copy_address(addr_ptr, addr, addr_len);

    nRF24L01_write_register(rf, RX_ADDR_P0 + pipe, addr, addr_len);    // reg 0x0A + pipe

    uint8_t current_pipes;
    nRF24L01_read_register(rf, EN_RXADDR, &current_pipes, 1);   // reg 02
    current_pipes |= _BV(pipe);                                 // поднимаем бит номер pipe
    nRF24L01_write_register(rf, EN_RXADDR, &current_pipes, 1);  // reg 02

    set_high(rf->ce);
}

bool nRF24L01_data_received(nRF24L01 *rf) {
//LV: -???    set_low(rf->ce);
//    nRF24L01_update_status(rf);
//    return nRF24L01_pipe_number_received(rf) >= 0;  // если FIFO не пуст (nRF24L01_pipe_number_received вернет номер пайпа)
//                                                    // то будет return TRUE;
//                                                    // иначе (nRF24L01_pipe_number_received вернет -1) будет return FALSE;
//                                                    // там же - msg.pipenumber
    return nRF24L01_update_status(rf) & _BV(RX_DR);
}

int nRF24L01_pipe_number_received(nRF24L01 *rf) {
    int pipe_number = (rf->status & RX_P_NO_MASK) >> 1;
    return pipe_number <= 5 ? pipe_number : -1;         // если ничего не принято (FIFO пуст), то в pipe_number будет 0b0111
                                                        // и будет return -1;
}

bool nRF24L01_read_received_data(nRF24L01 *rf, nRF24L01Message *message) {
    message->pipe_number = nRF24L01_pipe_number_received(rf);
//ORIG    nRF24L01_clear_receive_interrupt(rf);
    if (message->pipe_number < 0) {     // FIFO пуст, nRF24L01_pipe_number_received вернула -1
        message->length = 0;
        return false;
    }

    nRF24L01_read_register(rf, R_RX_PL_WID, &message->length, 1);   // считываем длину принятого пакета в &message->length

    if (message->length > 0) {
        nRF24L01_send_command(rf, R_RX_PAYLOAD, &message->data, message->length);
    }

//LV - переместил очистку сюда
    nRF24L01_clear_receive_interrupt(rf);

    return true;
}

void nRF24L01_clear_receive_interrupt(nRF24L01 *rf) {
    uint8_t data = _BV(RX_DR) | rf->status;
    nRF24L01_write_register(rf, STATUS, &data, 1);
}

void nRF24L01_transmit(nRF24L01 *rf, void *address, nRF24L01Message *msg) {
    nRF24L01_clear_transmit_interrupts(rf);
    uint8_t addr[5];
    uint8_t addr_len = 5;	//LV: Temporary written as 5
    copy_address((uint8_t *)address, addr, addr_len);
    nRF24L01_write_register(rf, TX_ADDR, addr, 5);
    copy_address((uint8_t *)address, addr, addr_len);
    nRF24L01_write_register(rf, RX_ADDR_P0, addr, 5);
    nRF24L01_send_command(rf, W_TX_PAYLOAD, &msg->data, msg->length);
    uint8_t config;
    nRF24L01_read_register(rf, CONFIG, &config, 1);
    config &= ~_BV(PRIM_RX);
    nRF24L01_write_register(rf, CONFIG, &config, 1);
    set_high(rf->ce);
}

uint8_t nRF24L01_write_register(nRF24L01 *rf, uint8_t reg_address, void *data, size_t length) {
    return nRF24L01_send_command(rf, W_REGISTER | reg_address, data, length);
}

uint8_t nRF24L01_read_register(nRF24L01 *rf, uint8_t reg_address, void *data, size_t length) {
    return nRF24L01_send_command(rf, R_REGISTER | reg_address, data, length);
}

uint8_t nRF24L01_no_op(nRF24L01 *rf) {
    return nRF24L01_send_command(rf, NOP, NULL, 0);
}

uint8_t nRF24L01_update_status(nRF24L01 *rf) {
    return nRF24L01_no_op(rf);
}

uint8_t nRF24L01_send_command(nRF24L01 *rf, uint8_t command, void *data, size_t length) {

    set_low(rf->ss);

    rf->status = spi_transfer(command);
    for (unsigned int i = 0; i < length; i++)	// If length=0 this cycle is not performed therefore data=NULL is not fatal
        ((uint8_t*)data)[i] = spi_transfer(((uint8_t*)data)[i]);

    set_high(rf->ss);

    return rf->status;
}

int nRF24L01_transmit_success(nRF24L01 *rf) {
    set_low(rf->ce);
    nRF24L01_update_status(rf);
    int success;
    if (rf->status & _BV(TX_DS)) success = 0;
    else if (rf->status & _BV(MAX_RT)) success = -1;
    else success = -2;
    nRF24L01_clear_transmit_interrupts(rf);
    uint8_t config;
    nRF24L01_read_register(rf, CONFIG, &config, 1);
    config |= _BV(PRIM_RX);
    nRF24L01_write_register(rf, CONFIG, &config, 1);
    return success;
}

void nRF24L01_flush_transmit_message(nRF24L01 *rf) {
    nRF24L01_send_command(rf, FLUSH_TX, NULL, 0);
}

void nRF24L01_retry_transmit(nRF24L01 *rf) {
    // XXX not sure it works this way, never tested
    uint8_t config;
    nRF24L01_read_register(rf, CONFIG, &config, 1);
    config &= ~_BV(PRIM_RX);
    nRF24L01_write_register(rf, CONFIG, &config, 1);
    set_high(rf->ce);
}

void nRF24L01_clear_interrupts(nRF24L01 *rf) {
    uint8_t data = _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT);
    nRF24L01_write_register(rf, STATUS, &data, 1);
}

void nRF24L01_clear_transmit_interrupts(nRF24L01 *rf) {
    uint8_t data = _BV(TX_DS) | _BV(MAX_RT);
    nRF24L01_write_register(rf, STATUS, &data, 1);
}

static void copy_address(uint8_t *source, uint8_t *destination, uint8_t addr_len) {
    for (int i = 0; i < addr_len; i++)
        destination[i] = source[i];
}

inline static void set_as_output(gpio_pin gpin) {
    volatile uint8_t *ddr = gpin.port - 1;
    *ddr |= _BV(gpin.pin);
}

inline static void set_as_input(gpio_pin gpin) {
    volatile uint8_t *ddr = gpin.port - 1;
    *ddr &= ~_BV(gpin.pin);
}

inline static void set_high(gpio_pin gpin) {
    *gpin.port |= _BV(gpin.pin);
}

inline static void set_low(gpio_pin gpin) {
    *gpin.port &= ~_BV(gpin.pin);
}

static void spi_init(nRF24L01 *rf) {
    // set as master
    SPCR |= _BV(MSTR);
    // enable SPI
    SPCR |= _BV(SPE);
    // MISO pin automatically overrides to input
    set_as_output(rf->sck);
    set_as_output(rf->mosi);
    set_as_input(rf->miso);
    // SPI mode 0: Clock Polarity CPOL = 0, Clock Phase CPHA = 0
    SPCR &= ~_BV(CPOL);
    SPCR &= ~_BV(CPHA);
    // Clock 2X speed
    SPCR &= ~_BV(SPR0);
//    SPCR &= ~_BV(SPR1);
    SPCR |= _BV(SPR1);
//    SPSR |= _BV(SPI2X);
    SPSR &= ~_BV(SPI2X);
    // most significant first (MSB)
//    SPCR &= ~_BV(DORD);
}

static uint8_t spi_transfer(uint8_t data) {
    SPDR = data;
    while (!(SPSR & _BV(SPIF)));
    return SPDR;
}
