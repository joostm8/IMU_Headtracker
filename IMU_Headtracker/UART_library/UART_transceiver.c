/*
 * UART_transceiver.c
 *
 * Created: 15/10/2018 15:19:49
 *  Author: Joost
 */ 

/****************************************************************************
Includes
****************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <string.h>
#include <util/delay.h>
#include "UART_transceiver.h"

/****************************************************************************
Macros
****************************************************************************/
#define UART_tx_enable()	UCSR0B |= 1<<TXEN0
#define UART_rx_enable()	UCSR0B |= 1<<RXEN0
#define UART_rx_disable()	UCSR0B &= ~(1<<RXEN0)
#define UART_enable()		UCSR0B |= 1<<TXEN0 | 1<<RXEN0

/****************************************************************************
Variables
****************************************************************************/
static uint32_t tx_length = 0;
static uint8_t* tx_buffer = 0;
static uint8_t* rx_buffer = 0;
typedef enum uart_state {
	waiting,
	tx,
	rx,
	tx_rx
} uart_state;
static volatile uart_state state = waiting;
static volatile uint32_t tx_cnt = 0;
static volatile uint32_t rx_cnt = 0;
static const unsigned char rx_terminator[2] = "\r\n";

/****************************************************************************
Call this function to set up the UART to its initial standby state.
Remember to enable interrupts from the main application after initializing the UART.
****************************************************************************/
void UART_initialise(){
	//initial UART config
	UCSR0A &= !(1<<U2X0);// clear this bit, not properly cleaned up from bootloader
	UCSR0B = 1<<RXCIE0 | 1<<TXCIE0 | 0<<UDRIE0 | 0<<UCSZ02;
	UCSR0C = 1<<UCSZ01 | 1<<UCSZ00;
	UBRR0 = 12;
	UART_tx_enable();
}

/****************************************************************************
Call this function to transmit a byte stream over the UART interface.
The function is blocking, but power optimized.

	tx_data_ptr	pointer to byte stream
	length		number of bytes to transmit
****************************************************************************/
uint8_t UART_tx(unsigned char* tx_data_ptr, uint32_t length){
	if(state == waiting){
		if(length > 0){
			state = tx;
			tx_length = length;
			tx_cnt = 0; //reset txCnt to 0
			tx_buffer = tx_data_ptr;
			UDR0 = *tx_buffer; //load first byte
			UCSR0B |= 1<<UDRIE0; 
			//enable interrupt, ISR takes over from here
			while(state == tx){
				cli();
				sleep_enable();
				sei();
				sleep_cpu();
				sleep_disable();
			}
			return 1;
		}
		return 0;
	}
	return 0;	
}

/****************************************************************************
Call this function to transmit a byte stream over the UART interface.
The response is buffered in rx_data_ptr, and assumes /cr/lf to be the
termination of the response. The function is blocking, but power optimized.

	tx_data_ptr	pointer to transmit byte stream
	rx_data_ptr pointer to reception buffer in which response is stored
				make sure to leave room for the \0 terminator.
	length		number of bytes to transmit
****************************************************************************/
uint8_t UART_tx_rx(unsigned char* tx_data_ptr, unsigned char* rx_data_ptr, uint32_t length){
	if(state == waiting){
		if(length > 0){
			state = tx;
			tx_length = length;
			tx_cnt = 0; // reset tx_cnt to 0
			rx_cnt = 0; // reset rx_cnt to 0
			tx_buffer = tx_data_ptr;
			rx_buffer = rx_data_ptr;
			UART_enable();
			UDR0 = *tx_buffer; //load first byte
			UCSR0B |= 1<<UDRIE0;
			//enable interrupt, ISR takes over from here
			while(state != waiting){
				cli();
				sleep_enable();
				sei();
				sleep_cpu();
				sleep_disable();
			}
			return 1;
		}
		return 0;
	}
	return 0;
}

/****************************************************************************
Call this function to transmit a byte stream over the UART interface.
The response is buffered in rx_data_ptr, and assumes /cr/lf to be the
termination of the response. A timeout can be specified in milliseconds.
The function is blocking, and not power optimized in order to facilitate
the timeout without timer. The timeout is not entirely accurate.

	tx_data_ptr	pointer to transmit byte stream
	rx_data_ptr pointer to reception buffer in which response is stored
				make sure to leave room for the \0 terminator.
	length		number of bytes to transmit3
	timeout		timeout after which to stop reception, in ms
****************************************************************************/
uint8_t UART_tx_rx_w_timeout(unsigned char* tx_data_ptr, unsigned char* rx_data_ptr, uint32_t length, uint32_t timeout){
	if(state == waiting){
		if(length > 0){
			int timeout_cnt = 0;
			state = tx;
			tx_length = length;
			tx_cnt = 0; // reset tx_cnt to 0
			rx_cnt = 0; // reset rx_cnt to 0
			tx_buffer = tx_data_ptr;
			rx_buffer = rx_data_ptr;
			UART_enable();
			UDR0 = *tx_buffer; //load first byte
			UCSR0B |= 1<<UDRIE0;
			//enable interrupt, ISR takes over from here
			while(state != waiting && timeout_cnt < timeout){
				++timeout_cnt;
				_delay_ms(1);
			}
			if(timeout_cnt >= timeout){
				//in this case return 0, reset states and set uart back to waiting
				UCSR0B &= ~(1<<UDRIE0); // disable transmit interrupt in case timeout < transmit time
				UART_rx_disable();
				state = waiting;
				return 0;
			}
			// otherwise just return 1;
			return 1;
		}
		return 0;
	}
	return 0;	
}

ISR(USART_RX_vect){
	rx_buffer[rx_cnt] = UDR0;
	if(rx_cnt > 0)
		if(memcmp(&rx_buffer[rx_cnt-1], rx_terminator, 2)){
			// In this case, we just append \0 and disable the UART reception, then set correct state
			rx_buffer[rx_cnt + 1] = '\0';
			if(state == tx_rx)
				state = tx;
			else
				state = waiting;
			UART_rx_disable();
		}
	++rx_cnt;
}

ISR(USART_UDRE_vect){
	++tx_cnt;// increment txCnt (since the byte has been loaded in transmitter
	if(tx_cnt < tx_length){
		UDR0 = tx_buffer[tx_cnt];
		//this clears the interrupt flag automatically.
	}
	else{
		//disable UDRE interrupt
		UCSR0B &= ~(1<<UDRIE0);
		//once last byte has left, TX_vect should trigger, updating the transmit state
	}
}

ISR(USART_TX_vect){
	// set state to only rx of coming from tx_rx
	// alternatively, when coming from tx, go back to waiting.
	if(state == tx_rx)
		state = rx;
	else
		state = waiting;
}

