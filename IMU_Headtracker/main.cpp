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

#include <avr/io.h>
#include "TWI_Master.h"
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
	
	// Init microcontroller
	
	// Init MPU9250
	
	// Init Bluetooth controller (if needed)
	
	/* Add calibration code here */
	
	// Calibrate magneto
	
	// Calibrate gyro
	
    while (1) 
    {
		/* Continuously update Bluetooth controller with axes info*/
    }
}

