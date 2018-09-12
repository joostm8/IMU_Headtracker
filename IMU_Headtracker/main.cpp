/*
 * IMU_Headtracker.c
 *
 * Created: 1/08/2018 20:43:46
 * Author : Joost
 *
 * IMU headtracker
 * Requires:
 * MPU9250
 * Atmega328p
 * rn42 (or hc05 flashed as rn42)
 * 
 */ 

#define F_CPU 8000000UL

#define MPU_9250_Addr 0b1101000 // 7 bit I2C address with AD0 pin to ground

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
extern "C"{
#include "TWI_Master.h"	
	};
#include "MahonyAHRS.h"

static enum programState{
	calibration,
	operation
	} pState = calibration;
	
static enum bluetoothState{
	startup,
	ready,
	connected
	} btState = startup;

int main(void)
{
    /* Replace with your application code */
	
	/* Add startup code here*/
	// Start I2C interface using provided library by microchip
	// in TWI_Master.h, TWI_TWBR is set as 0x02, which yields 400kHz on 8MHz µC
	TWI_Master_Initialise();
	// The default MPU9250 settings suffice, which is 250 DPS for the gyro and 2G for acc.
	// seems unfeasible that clipping would occur when just attached to your head
	
	// Init UART
	// A baudrate of 38.4k is the highest achievable baudrate given a µC at 8 MHz
	/* transmit function must enable UDRE interrupt, load first byte,
		then UDRE can be used to load following bytes, when no bytes available,
		disable UDRE interrupt in ISR, otherwise interrupt keeps executing.
		TXC can be used to reset state of the transmit function (i.e. accept new transmit)
	*/
	//initial UART config
	UCSR0B = 1<<RXCIE0 | 1<<TXCIE0 | 0<<UDRIE0 | 0<<UCSZ02;
	UCSR0C = 1<<UCSZ01 | 1<<UCSZ00;
	UBRR0 = 12;
	// enable uart
	UCSR0B |= 1<<RXEN0 | 1<<TXEN0;
	// enable global interrupts such that the I2C and UART interfaces can operate
	sei();
	// Init (calibrate, init is ok) MPU9250
	_delay_ms(500); // replace with some code that turns on an LED so you know when to place the device on a flat surface
	// Gyroscope is easy, perform a couple of measurements, sum average over the duration, set registers.
	// Accelerometer: first read the internal offsets, then sample and average, finally subtract sampled values from read values and set.
	// Magnetometer: has no internal calibartion, just read the scaling settings. Hard iron and Soft iron effects must be compensated for in software.
	
	// Init Bluetooth controller (if needed)
	
    while (1) 
    {
		/* Continuously update Bluetooth controller with axes info*/
    }
}

ISR(USART_RX_vect){
	
}

ISR(USART_UDRE_vect){
	
}

ISR(USART_TX_vect){
	
}

