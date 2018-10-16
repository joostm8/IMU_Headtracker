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

/****************************************************************************
Macros
****************************************************************************/
#define MPU_9250_Addr 0b1101000 // 7 bit I2C address with AD0 pin to ground

/****************************************************************************
Includes
****************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
extern "C"{
#include "TWI_Master.h"
#include "UART_transceiver.h"
	};
#include "MahonyAHRS.h"

/****************************************************************************
Variables
****************************************************************************/
static enum programState{
	calibration,
	operation
	} pState = calibration;
	
static enum bluetoothState{
	startup,
	ready,
	connected
	} btState = startup;

static float hard_iron_correction[3] = {0, 0, 0}; //x, y, z hard iron corrections
static float soft_iron_correction[3] = {0, 0, 0}; //x, y, z soft iron corrections
// also see https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
static uint8_t magneto_sens_adjust[3] = {0, 0, 0}; // x, y, z sens adjust values
// perhaps store sens adjust in eeprom since it doesn't change?
/****************************************************************************
  Function declarations
****************************************************************************/
uint8_t MPU_9250_initialise();
uint8_t calibrate_accelerometer();
uint8_t read_accel_bias(int16_t*);
int8_t set_accel_bias(int16_t*);
int8_t set_gyro_bias(int16_t*);
uint8_t calibrate_gyroscope();
uint8_t calibrate_magnetometer();
uint8_t scale_magnetometer();

/****************************************************************************
  Main code
****************************************************************************/
int main(void)
{
	// Start I2C interface using provided library by microchip
	// in TWI_Master.h, TWI_TWBR is set as 0x02, which yields 400kHz on 8MHz µC
	TWI_Master_Initialise();
		
	// Initialise UART at 38.4kbaud, the highest baudrate given F_CPU = 8 MHz
	UART_initialise();
	
	// enable global interrupts such that the I2C and UART interfaces can operate
	sei();
	
	// Initialise and calibrate MPU9250;
	MPU_9250_initialise();
	calibrate_accelerometer;
	calibrate_gyroscope();
	// put some code here so you know when to do the 8 pattern.
	calibrate_magnetometer();
	
	
	// Magnetometer: has no internal calibartion, just read the scaling settings. Hard iron and Soft iron effects must be compensated for in software.
	
	// Init Bluetooth controller (if needed)
	
    while (1) 
    {
		/* Continuously update Bluetooth controller with axes info*/
    }
}

/****************************************************************************
Call this function to initialise the MPU_9250, just selects clock src.
****************************************************************************/
uint8_t MPU_9250_initialise(){
	uint8_t write_cmd[3];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 107; // PWR_MGMT_1
	write_cmd[2] = 0x01; // auto select clock source;
}

/****************************************************************************
Call this function to calibrate the accelerometer.
It calculates average offsets when the device is on a flat surface, and
writes these offsets back to the device.
****************************************************************************/
uint8_t calibrate_accelerometer(){
	// Set accelerometer range to +/-16g since offset is stored in that format
	uint8_t write_cmd[3];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 28; // Accelerometer configuration register
	write_cmd[2] = 0b00011000; // set ACCEL_FS_SEL to 11 or +/- 16g;
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	
	// take 1024 samples to average
	uint8_t read_cmd[7];
	int32_t sum[3];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 59; // ACCEL_XOUT_H;
	
	read_cmd[0] = MPU_9250_Addr << 1 | 1; // read
	for(uint16_t i = 0; i < 1024; ++i){
		TWI_Start_Transceiver_With_Data(write_cmd, 2);
		TWI_Start_Transceiver_With_Data(read_cmd, 7);
		TWI_Get_Data_From_Transceiver(read_cmd, 7);
		
		//accumulate immediately to limit RAM usage
		sum[0] += read_cmd[1] << 8 | read_cmd[2];
		sum[1] += read_cmd[3] << 8 | read_cmd[4];
		sum[2] += read_cmd[5] << 8 | read_cmd[6];
		
		//update rate is 1kHz with default settings
		_delay_ms(1);
	}
	
	//take averages, sign inversion is done when setting the biases
	//note that integer accuracy loss isn't bad because
	//the values are stored as integers on the MPU anyway.
	int16_t accel_bias[3];
	accel_bias[0] = (int16_t)(sum[0]/1024); //x
	accel_bias[1] = (int16_t)(sum[1]/1024); //y
	accel_bias[2] = (int16_t)(sum[2]/1024); //z, from which you must subtract/add 2048 (1g) since z axis measures 1g
	if(accel_bias[2] > 0)
		accel_bias[2] -= 2048; //if z value is positive, subtract 1g;
	else
		accel_bias[2] += 2048; //otherwise, add 1g;
	// note that the if else above does not expect more than 1g of offset (shouldn't be the case)
	
	set_accel_bias(accel_bias);
	
	return 0;
}

/****************************************************************************
Reads the factory offsets from the device (only needed for accelerometer
****************************************************************************/
uint8_t read_accel_bias(int16_t* accel_bias){
	uint8_t cmd[7]; // I2C command
	cmd[0] = MPU_9250_Addr << 1; // write
	cmd[1] = 119; //XA_OFFSET_H;
	TWI_Start_Transceiver_With_Data(cmd, 2);
	
	memset(cmd, 0, 7);
	cmd[0] = (MPU_9250_Addr << 1) | 1; // read
	TWI_Start_Transceiver_With_Data(cmd, 7);// addr + R, 6 registers
	TWI_Get_Data_From_Transceiver(cmd, 7); // reads out the received bytes in the empty spaces
	accel_bias[0] = cmd[1] << 8 | cmd[2];
	accel_bias[1] = cmd[3] << 8 | cmd[4];
	accel_bias[2] = cmd[5] << 8 | cmd[6];
	
	return 0;
}

/****************************************************************************
This function writes accelerometer biases to the offset registers. The biases
are relative to the current output and are added to the factory values.
LSB in +/- 16g format
****************************************************************************/
int8_t set_accel_bias(int16_t* accel_bias){
	uint8_t data[6];
	int16_t accel_reg_bias[3];
	int16_t mask = 0x0001;
	int8_t mask_bit[3];
	
	if(read_accel_bias(accel_reg_bias))
		return -1;
	
	//bit 0 of the 2 byte bias is for temp comp
	//calculations need to compensate for this
	for(uint8_t i = 0; i<3; ++i){
		if(accel_reg_bias[i] & mask)
			mask_bit[i] = 1;
	}
	
	accel_reg_bias[0] -= accel_bias[0];
	accel_reg_bias[1] -= accel_bias[1];
	accel_reg_bias[2] -= accel_bias[2];
	
	data[0] = (accel_reg_bias[0] >> 8) & 0xff;
	data[1] = (accel_reg_bias[0]) & 0xff;
	data[1] = data[1]|mask_bit[0];
	data[2] = (accel_reg_bias[1] >> 8) & 0xff;
	data[3] = (accel_reg_bias[1]) & 0xff;
	data[3] = data[3]|mask_bit[1];
	data[4] = (accel_reg_bias[2] >> 8) & 0xff;
	data[5] = (accel_reg_bias[2]) & 0xff;
	data[5] = data[5]|mask_bit[2];
	
	uint8_t cmd[8];
	cmd[0] = MPU_9250_Addr << 1; //Write
	cmd[1] = 119; //XA_OFFSET_H;
	memcpy(&cmd[2], data, 6);// copy data over
	TWI_Start_Transceiver_With_Data(cmd, 8);
	
	return 0;
}

/****************************************************************************
Call this function to calibrate the gyroscope.
It calculates average offsets when the device is on a flat surface, and
writes these offsets back to the device.
****************************************************************************/
uint8_t calibrate_gyroscope(){
	// Set gyroscope range to 1000 dps since offset is stored in that format
	uint8_t write_cmd[3];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 27; // Gyro configuration register
	write_cmd[2] = 0b00010000; // set GYRO_FS_SEL to 10 or 1000 dps;
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	
	// take 1024 samples to average
	uint8_t read_cmd[7];
	int32_t sum[3];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 67; // GYRO_XOUT_H;
	
	read_cmd[0] = MPU_9250_Addr << 1 | 1; // read
	for(uint16_t i = 0; i < 1024; ++i){
		TWI_Start_Transceiver_With_Data(write_cmd, 2);
		TWI_Start_Transceiver_With_Data(read_cmd, 7);
		TWI_Get_Data_From_Transceiver(read_cmd, 7);
		
		//accumulate immediately to limit RAM usage
		sum[0] += read_cmd[1] << 8 | read_cmd[2];
		sum[1] += read_cmd[3] << 8 | read_cmd[4];
		sum[2] += read_cmd[5] << 8 | read_cmd[6];
		
		//update rate is 8kHz with default settings
		_delay_us(125);
	}
	
	//take averages, sign inversion is done when setting the biases
	//note that integer accuracy loss isn't bad because
	//the values are stored as integers on the MPU anyway.
	int16_t gyro_bias[3];
	gyro_bias[0] = (int16_t)(sum[0]/1024); //x
	gyro_bias[1] = (int16_t)(sum[1]/1024); //y
	gyro_bias[2] = (int16_t)(sum[2]/1024); //z
	
	set_gyro_bias(gyro_bias);
	
	return 0;
}

/****************************************************************************
This function writes gyroscope biases to the offset registers.
1000 dps format
****************************************************************************/
int8_t set_gyro_bias(int16_t* gyro_bias){
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
		
	for(uint8_t i = 0 ; i < 3 ; ++i) {
		gyro_bias[i] = (-gyro_bias[i]);
	}
	
	data[0] = (gyro_bias[0] >> 8) & 0xff;
	data[1] = (gyro_bias[0]) & 0xff;
	data[2] = (gyro_bias[1] >> 8) & 0xff;
	data[3] = (gyro_bias[1]) & 0xff;
	data[4] = (gyro_bias[2] >> 8) & 0xff;
	data[5] = (gyro_bias[2]) & 0xff;
	
	uint8_t cmd[8];
	cmd[0] = MPU_9250_Addr << 1; //Write
	cmd[1] = 19; //XG_OFFSET_H;
	memcpy(&cmd[2], data, 6);// copy data over
	TWI_Start_Transceiver_With_Data(cmd, 8);
	
	return 0;
}

uint8_t calibrate_magnetometer(){

}

uint8_t scale_magnetometer(){
	
}
