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
#define AK8963_addr 0x0C // 7 bit magnetometer address
#define GYROSCOPE_RES_RPS 0.0005321125 // rps/lsb

// Timer 1 (16 bit) will be set to increment at rate of 1 �s
// this macro therefore allows for time deltas of up to 65 ms to be measured.
#define micros() TCNT1 

/****************************************************************************
Includes
****************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "TWI_Master.h"
#include "UART_transceiver.h"
#include "MadgwickAHRS.h"

/****************************************************************************
Variables
****************************************************************************/
static int16_t hard_iron_correction[3] = {0, 0, 0}; //x, y, z hard iron corrections, subtract from scaled measurements
// also see https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
// precomputed magnetometer scale * soft iron
static float scale_times_soft_iron[3] = {0, 0, 0};
static uint8_t magneto_sens_adjust[3] = {0, 0, 0}; // x, y, z sens adjust values
static int16_t accelerometer_offset[3] = {0, 0, 0}; // accelerometer offsets to handle offsetting in SW.

// extern variables used in by MahonyAHRS
extern volatile float roll, pitch, yaw;
extern volatile float dt;	
extern volatile float q0, q1, q2, q3;

static uint8_t BT_command[8] = {0xFD, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // BT command for HID game controller
static uint8_t send_BT_cmd = 0; // true/false when BT_cmd may be transmitted to BT chip

/****************************************************************************
  Function declarations
****************************************************************************/
void MPU_9250_initialise();
void calibrate_accelerometer();
void set_gyro_bias(int16_t*);
void calibrate_gyroscope();
void calibrate_magnetometer();
void get_magnetometer_scale();
uint8_t read_magnetometer(int16_t*);
void scale_magnetometer(float*);
void read_accelerometer_data(int16_t*);
void read_gyroscope_data(int16_t*);
void apply_magnetometer_scaling(int16_t*, float*);
void apply_accelerometer_offset(int16_t*);
void GPIO_setup_for_timing();
void GPIO_initialise();
void TIMER_setup();

/****************************************************************************
  Main code
****************************************************************************/

int main(void)
{
	
	// Start I2C interface using provided library by microchip
	// in TWI_Master.h, TWI_TWBR is set as 0x02, which yields 400kHz on 8MHz �C
	TWI_Master_Initialise();
		
	// Initialise UART at 38.4kbaud, the highest baudrate given F_CPU = 8 MHz
	UART_initialise();
	
	// Initialise LED GPIO
	GPIO_initialise();
	
	// enable global interrupts such that the I2C and UART interfaces can operate
	sei();

	_delay_ms(1000);
	
	// Initialise and calibrate MPU9250;
	MPU_9250_initialise();
	
	// Turn on pin 7, indicates to user to put the device on a flat surface
	PORTD |= 1 << PORTD7;
	
	_delay_ms(3000); // 3 second delay for user to put the device down.
	
	// Calibrate accelerometer;
	calibrate_accelerometer();
	
	// Calibrate gyroscope
	calibrate_gyroscope();
	
	// Turn off pin 7 LED
	PORTD &= ~(1 << PORTD7);
	// Turn on pin 6 LED, indicates to start waving 8 pattern
	PORTD |= 1 << PORTD6;
	
	_delay_ms(1000); // 1 sec delay for user response time.
	
	// calibrate magnetometer	
	get_magnetometer_scale();
	
	calibrate_magnetometer();
	
	// turn off LED, so user can stop waving
	PORTD &= ~(1 << PORTD6);

	//gyro is by now set to 1000 dps, which is ok
	//magneto is also set in +- 4g so we can move on to main program
	
	// Init Bluetooth controller (if needed)
	float magnetometer_data_float[3];
	float gyroscope_data_rps[3];
	int16_t magnetometer_data_raw[3];
	int16_t gyroscope_data_raw[3];
	int16_t accelerometer_data[3];
	TIMER_setup();
	GPIO_setup_for_timing();
	
	uint16_t start = micros();
	uint16_t stop = 0;
	uint8_t overflow = 0;
	
	// algorithm will run as fast as possible.
    while (1) 
    {
			PORTB |= 1<<PORTB1;
			read_gyroscope_data(gyroscope_data_raw);

			read_accelerometer_data(accelerometer_data);
			
			apply_accelerometer_offset(accelerometer_data);

			overflow = read_magnetometer(magnetometer_data_raw);
				
			//scale_magnetometer(magnetometer_data_gaus);
			if(!overflow){
				apply_magnetometer_scaling(magnetometer_data_raw, magnetometer_data_float);
			}
			else{
				magnetometer_data_float[0] = 0.0f;
				magnetometer_data_float[1] = 0.0f;
				magnetometer_data_float[2] = 0.0f;
			}
			
	
			//conversion to appropriate datatypes happens here
			gyroscope_data_rps[0] = (GYROSCOPE_RES_RPS * (float)gyroscope_data_raw[0]);
			gyroscope_data_rps[1] = (GYROSCOPE_RES_RPS * (float)gyroscope_data_raw[1]);
			gyroscope_data_rps[2] = (GYROSCOPE_RES_RPS * (float)gyroscope_data_raw[2]);
			
			// Note: for the Mahony algorithm: gx, gy, gz in radians/second
			// accelerometer data and magnetometer data can be in ANY calibrated unit (so even LSB)
			// magnetometer data must be scaled anyhow, but there is no point in converting to Tesla or Gauss
			
			stop = micros(); // get stop time before updating mahony
			dt = stop > start ? ((float)(stop - start))/1000000.0f : ((float)(UINT16_MAX - (start - stop)))/1000000.0f;
			start = micros(); // get start time for time delta for next calculation.
			
			MadgwickAHRSupdate( -gyroscope_data_rps[0],
								gyroscope_data_rps[1],
								-gyroscope_data_rps[2],
								(float)(-accelerometer_data[0]),
								(float)(accelerometer_data[1]),
								(float)(-accelerometer_data[2]),
								-magnetometer_data_float[1],
								magnetometer_data_float[0],
								magnetometer_data_float[2]);
					
			MadgwickAHRSupdateRollPitchYaw(); // move this within the BT transmit code perhaps since it's only needed there?.
			// update Bluetooth controller with axes info
			// lets start by just printing to serial :)

		// update BT module at approximately 60 Hz (can't find max supported rate, but at full blast, the BT module experienced crashes)
		if(send_BT_cmd){
			send_BT_cmd = 0;
			BT_command[2] = (int8_t)roll;	//X1
			BT_command[3] = (int8_t)pitch;	//Y1
			BT_command[4] = (int8_t)yaw;	//X2
			UART_tx((unsigned char*)BT_command, 8);
		}
			PORTB &= ~(1<<PORTB1);	
    }
	return 0;
}

/****************************************************************************
Call this function to initialise the MPU_9250, just selects clock src.
****************************************************************************/
void MPU_9250_initialise(){
	//configure clock source
	uint8_t write_cmd[3];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 107; // PWR_MGMT_1
	write_cmd[2] = 0x80; // reset registers (so a reset of the MCU also cleans IMU calibrations)
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	
	write_cmd[1] = 107; // PWR_MGMT_1
	write_cmd[2] = 0x01; // auto select clock source;
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	
	//enable pass-through mode (also bypass mode) to access AK8963 (magneto)
	memset(write_cmd, 0, 3);
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 55; // INT_PIN_CFG
	write_cmd[2] = 0x02; // BYPASS_EN
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
}

/****************************************************************************
Call this function to calibrate the accelerometer.
It calculates average offsets when the device is on a flat surface, and
writes these offsets back to the device.
****************************************************************************/
void calibrate_accelerometer(){
	// Set accelerometer range to +/- 4g since I expect this range to be used
	uint8_t write_cmd[3];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 28; // Accelerometer configuration register
	write_cmd[2] = 0b00001000; // set ACCEL_FS_SEL to 01 or +/- 4g, DLPC_CFG to 0 (250 Hz)
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	
	// take 1024 samples to average
	int32_t sum[3] = {0, 0, 0};
	int16_t accelerometer_data[3] = {0, 0, 0};
	for(uint16_t i = 0; i < 2048; ++i){
		read_accelerometer_data(accelerometer_data);
		
		//accumulate immediately to limit RAM usage
		sum[0] += accelerometer_data[0];
		sum[1] += accelerometer_data[1];
		sum[2] += accelerometer_data[2];
		
		//update rate is 1kHz with default settings
		_delay_ms(1.1);
	}
	
	// int32_t is a signed long int, so when formatting in formatting functions
	// one needs to use %ld... Finally solved.
	sum[0] = (sum[0])/2048;
	sum[1] = (sum[1])/2048;
	sum[2] = (sum[2])/2048;
	
	//take averages, sign inversion is done when setting the biases
	//note that integer accuracy loss isn't bad because
	//the values are stored as integers on the MPU anyway.
	accelerometer_offset[0] = sum[0]; //x
	accelerometer_offset[1] = sum[1]; //y
	accelerometer_offset[2] = sum[2]; //z, from which you must subtract/add 2048 (1g) since z axis measures 1g
	if(accelerometer_offset[2] > 0)
		accelerometer_offset[2] -= 8192; //if z value is positive, subtract 1g (8192 LSB);
	else
		accelerometer_offset[2] += 8192; //otherwise, add 1g (8192 LSB);
	// note that the if else above does not expect more than 1g of offset (shouldn't be the case)
}

/****************************************************************************
Applies accelerometer offsets on data
****************************************************************************/
void apply_accelerometer_offset(int16_t* accel_data){
	accel_data[0] -= accelerometer_offset[0];
	accel_data[1] -= accelerometer_offset[1];
	accel_data[2] -= accelerometer_offset[2];
}

/****************************************************************************
Reads accelerometer data, assumes data is being collected continuously
****************************************************************************/
void read_accelerometer_data(int16_t* accel_data){
	uint8_t write_cmd[3];
	uint8_t read_cmd[7];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 59; // ACCEL_XOUT_H;	
	read_cmd[0] = MPU_9250_Addr << 1 | 1; // read
	TWI_Start_Transceiver_With_Data(write_cmd, 2);
	TWI_Start_Transceiver_With_Data(read_cmd, 7);
	TWI_Get_Data_From_Transceiver(read_cmd, 7);
	accel_data[0] = read_cmd[1] << 8 | read_cmd[2];
	accel_data[1] = read_cmd[3] << 8 | read_cmd[4];
	accel_data[2] = read_cmd[5] << 8 | read_cmd[6];
}

/****************************************************************************
Call this function to calibrate the gyroscope.
It calculates average offsets when the device is on a flat surface, and
writes these offsets back to the device.
****************************************************************************/
void calibrate_gyroscope(){
	// Set gyroscope range to 1000 dps since offset is stored in that format
	uint8_t write_cmd[3];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 27; // Gyro configuration register
	write_cmd[2] = 0b00010000; // set GYRO_FS_SEL to 10 or 1000 dps;
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	
	int32_t sum[3] = {0, 0, 0};
	int16_t gyro_data[3];
	// take 1024 samples to average
	for(uint16_t i = 0; i < 2048; ++i){
		//accumulate immediately to limit RAM usage
		read_gyroscope_data(gyro_data);
		sum[0] += gyro_data[0];
		sum[1] += gyro_data[1];
		sum[2] += gyro_data[2];	
		
		//update rate is 8kHz with default settings
		_delay_us(125);
	}
	
	//take averages, sign inversion is done when setting the biases
	//note that integer accuracy loss isn't bad because
	//the values are stored as integers on the MPU anyway.
	int16_t gyro_bias[3];
	gyro_bias[0] = (int16_t)(sum[0]/2048); //x
	gyro_bias[1] = (int16_t)(sum[1]/2048); //y
	gyro_bias[2] = (int16_t)(sum[2]/2048); //z
	
	set_gyro_bias(gyro_bias);
}

/****************************************************************************
This function reads gyroscope data.
It assumes data is being sampled
****************************************************************************/
void read_gyroscope_data(int16_t* gyro_data){
	uint8_t write_cmd[2];
	uint8_t read_cmd[7];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 67; // GYRO_XOUT_H;
	read_cmd[0] = MPU_9250_Addr << 1 | 1; // read
	TWI_Start_Transceiver_With_Data(write_cmd, 2);
	TWI_Start_Transceiver_With_Data(read_cmd, 7);
	TWI_Get_Data_From_Transceiver(read_cmd, 7);
	gyro_data[0] = read_cmd[1] << 8 | read_cmd[2];
	gyro_data[1] = read_cmd[3] << 8 | read_cmd[4];
	gyro_data[2] = read_cmd[5] << 8 | read_cmd[6];
}

/****************************************************************************
This function writes gyroscope biases to the offset registers.
1000 dps format
****************************************************************************/
void set_gyro_bias(int16_t* gyro_bias){
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
}

/****************************************************************************
This function gets the hard and soft iron offsets used to correct magnetometer
values.
It assumes the magneto is in power down mode at the start.
****************************************************************************/
void calibrate_magnetometer(){
	// set magnetometer to 16 bit continuous 100 Hz mode.
	uint8_t write_cmd[3];
	write_cmd[0] = AK8963_addr << 1; //Write
	write_cmd[1] = 0x0A; // CNTL1
	write_cmd[2] = 0x16; // 16 bit, Continuous 2 (100 Hz)
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	//The device is now continuously measuring the data.
	
	// at 100 Hz, a new sample is available every 10 ms.
	int16_t magneto_data[3];
	int16_t max[3] = {-32768, -32768, -32768};
	int16_t min[3] = {32767, 32767, 32767};
	// first goal is to determine the max and min x, y, z values
	for(uint16_t i = 0; i < 2048; ++i){
		
		read_magnetometer(magneto_data);
				
		for(uint8_t j = 0; j < 3; ++j){
			if(magneto_data[j] < min[j])
				min[j] = magneto_data[j];
			if(magneto_data[j] > max[j])
				max[j] = magneto_data[j];
		}
		_delay_ms(10);
	}
		
	// now calculate raw hard iron corrections:
	hard_iron_correction[0] = (max[0] + min[0])/2;
	hard_iron_correction[1] = (max[1] + min[1])/2;
	hard_iron_correction[2] = (max[2] + min[2])/2;
	
	// calculate soft iron correction (at once in float since I must scale them afterwards)
	float soft_iron_raw[3];
	soft_iron_raw[0] = (float)((max[0] - min[0])/2);
	soft_iron_raw[1] = (float)((max[1] - min[1])/2);
	soft_iron_raw[2] = (float)((max[2] - min[2])/2);

	// scale soft iron
	// not reallt needed to scale it here, could save 2 floating point multiplications
	float* soft_iron_scaled = soft_iron_raw;
	scale_magnetometer(soft_iron_scaled);
	
	// scaling only needs to be done here on avg_rad
	float avg_rad = soft_iron_scaled[0] + soft_iron_scaled[1] + soft_iron_scaled[2];
	avg_rad /= 3.0;
	
	float soft_iron_correction[3] = {0, 0, 0};
	soft_iron_correction[0] = avg_rad/soft_iron_scaled[0];
	soft_iron_correction[1] = avg_rad/soft_iron_scaled[1];
	soft_iron_correction[2] = avg_rad/soft_iron_scaled[2];
	
	scale_times_soft_iron[0] = (((float)magneto_sens_adjust[0] - 128.0) / 256.0 + 1.0) * soft_iron_correction[0];
	scale_times_soft_iron[1] = (((float)magneto_sens_adjust[1] - 128.0) / 256.0 + 1.0) * soft_iron_correction[1];
	scale_times_soft_iron[2] = (((float)magneto_sens_adjust[2] - 128.0) / 256.0 + 1.0) * soft_iron_correction[2];
}

/****************************************************************************
This function scales magnetometer readings with the scale adjustment values.
scaling happens in place
****************************************************************************/
void scale_magnetometer(float* magneto_data){
	magneto_data[0] = magneto_data[0] * (((float)magneto_sens_adjust[0] - 128) * 0.5 / 128.0 + 1.0);
	magneto_data[1] = magneto_data[1] * (((float)magneto_sens_adjust[1] - 128) * 0.5 / 128.0 + 1.0);
	magneto_data[2] = magneto_data[2] * (((float)magneto_sens_adjust[2] - 128) * 0.5 / 128.0 + 1.0);
}

/****************************************************************************
This function reads the magnetometer data. parameter is empty array of x, y, z
The function assumes the magnetometer is in a measurement mode

returns: 1 when magnetic overflow occurred
		 0 when no overflow occurred
****************************************************************************/
uint8_t read_magnetometer(int16_t* magneto_data){
	
	uint8_t write_cmd[2];
	write_cmd[0] = AK8963_addr << 1; //Write
	write_cmd[1] = 0x03; // HXL
	uint8_t read_cmd[8]; // one must also read status 2 register as it denotes the end of a read
	// further it contains the magnetic sensor overflow bit, which must be checked to know if algorithm
	// may use the magnetometer readings or not.
	read_cmd[0] = AK8963_addr << 1 | 1; // Read
			
	TWI_Start_Transceiver_With_Data(write_cmd, 2);
	TWI_Start_Transceiver_With_Data(read_cmd, 8);
	TWI_Get_Data_From_Transceiver(read_cmd, 8);
	
	magneto_data[0] = read_cmd[2] << 8 | read_cmd[1];
	magneto_data[1] = read_cmd[4] << 8 | read_cmd[3];
	magneto_data[2] = read_cmd[6] << 8 | read_cmd[5];
	
	// magneto data[7] is status reg 2
	return (magneto_data[7] & 0x08) > 0;
}

/****************************************************************************
This function gets the magnetometer scale adjustment, used later on in the
program.
It assumes the magneto is in power down mode at the start.
****************************************************************************/
void get_magnetometer_scale(){
	//First set magnetometer in Fuse ROM Access mode
	uint8_t write_cmd[3];
	write_cmd[0] = AK8963_addr << 1; //Write
	write_cmd[1] = 0x0A; // CNTL1
	write_cmd[2] = 0x0F; // Fuse ROM Access Mode
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	//The device is now in Fuse ROM Access Mode
	//Read the magneto sens adjust (int8)
	memset(write_cmd, 0, 3);
	write_cmd[0] = AK8963_addr << 1; //Write
	write_cmd[1] = 0x10; // X-axis sens adjust value
	uint8_t read_cmd[4];
	read_cmd[0] = AK8963_addr << 1 | 1; //Read
	TWI_Start_Transceiver_With_Data(write_cmd, 2);
	TWI_Start_Transceiver_With_Data(read_cmd, 4);
	TWI_Get_Data_From_Transceiver(read_cmd, 4);
	
	magneto_sens_adjust[0] = read_cmd[1];
	magneto_sens_adjust[1] = read_cmd[2];
	magneto_sens_adjust[2] = read_cmd[3];

	//Finally put the magnetometer back in power down mode
	memset(write_cmd, 0, 3);
	write_cmd[0] = AK8963_addr << 1; //Write
	write_cmd[1] = 0x0A; // CNTL1
	write_cmd[2] = 0x00; // Power Down Mode
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	// datasheet asks to wait 100 �s before transitioning to other modes,
	// delay here to be sure this is respected
	_delay_us(100);
}

/****************************************************************************
Applies  hard iron, soft iron and scaling correction on magnetometer readings.

normal fomula is:

data in Gauss (or Tesla) = (raw * scale - hard_iron * scale) * soft_iron * Res
with raw in LSB, hard_iron in lsb, scale the scalar scale factor per datasheet
soft iron as a scalar and Res either in Gauss/LSB or Tesla/LSB

Since we don't need to convert to Gauss or Tesla and we can bring scale upfront:
data in LSB = (raw - hard_iron) * scale * soft_iron
scale * soft_iron can be computed upfront and stored in variable scale_times_soft_iron
****************************************************************************/
void apply_magnetometer_scaling(int16_t* data_raw, float* data_calibrated){
	data_calibrated[0] = (data_raw[0] - hard_iron_correction[0]) * scale_times_soft_iron[0];
	data_calibrated[1] = (data_raw[1] - hard_iron_correction[1]) * scale_times_soft_iron[1];
	data_calibrated[2] = (data_raw[2] - hard_iron_correction[2]) * scale_times_soft_iron[2];
}

/****************************************************************************
Sets up TIMER1 (16 bits) in normal mode at 1 �s steps
****************************************************************************/
void TIMER_setup(){
	// setup in normal mode, but do enable OCR1A and COMPA interrupt
	OCR1A = 16665;
	TIFR1 = 0x02; // clear any pending interrupts first
	TIMSK1 |= 0x02; // enable OCIE1A
	TCCR1B |= 0x02; // select /8 prescaler and start timer
}

/****************************************************************************
GPIO pin for debug timing purposes.
****************************************************************************/
void GPIO_setup_for_timing(){
	DDRB |= 1 << PORTB1;	
}

/****************************************************************************
Setup pins 6 and 7 for LED output
****************************************************************************/
void GPIO_initialise(){
	DDRD |= (1 << PORTD7) | (1 << PORTD6);
}

/****************************************************************************
ISR for TIMER1 compare match event
****************************************************************************/
ISR(TIMER1_COMPA_vect){
	//The update rate won't be exactly 60 Hz since there will be some jitter
	// due to the transmission not occuring instantly, only at the end of
	// the main loop.
	send_BT_cmd = 1;
	OCR1A += 16665;//increment next output compare value.
	//since OCR1A is only 16 bits, I handily use the fact that it overflows
}

/****************************************************************************
Original accelerometer calibration follows. Unfortunately doesn't work
correctly. Main culprit seems to be the temperature adjust bit when writing
data to the offset registers. Alternatively I've used software offsetting
in the rest of the program.
****************************************************************************/

/****************************************************************************
Call this function to calibrate the accelerometer.
It calculates average offsets when the device is on a flat surface, and
writes these offsets back to the device.
****************************************************************************/
/*
uint8_t calibrate_accelerometer(){
	// Set accelerometer range to +/-16g since offset is stored in that format
	uint8_t write_cmd[3];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 28; // Accelerometer configuration register
	write_cmd[2] = 0b00011000; // set ACCEL_FS_SEL to 11 or +/- 16g;
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	
	_delay_ms(100);
	
	// take 1024 samples to average
	
	int32_t sum[3] = {0, 0, 0};
	int16_t accelerometer_data[3] = {0, 0, 0};
	for(uint16_t i = 0; i < 1024; ++i){
		read_accelerometer_data(accelerometer_data);
		
		//accumulate immediately to limit RAM usage
		sum[0] += accelerometer_data[0];
		sum[1] += accelerometer_data[1];
		sum[2] += accelerometer_data[2];
		
		//update rate is 1kHz with default settings
		_delay_ms(2);
	}
	
	sprintf(info, "Sum ax: %ld\r\n", sum[0]);
	UART_tx((unsigned char*)info, strlen(info));
	memset(info, 0, 50);
	sprintf(info, "Sum ay: %ld\r\n", sum[1]);
	UART_tx((unsigned char*)info, strlen(info));
	memset(info, 0, 50);
	sprintf(info, "Sum az: %ld\r\n", sum[2]);
	UART_tx((unsigned char*)info, strlen(info));
	memset(info, 0, 50);
	
	// int32_t is a signed long int, so when formatting in formatting functions
	// one needs to use %ld... Finally solved.
	sum[0] = (sum[0])/1024;
	sum[1] = (sum[1])/1024;
	sum[2] = (sum[2])/1024;
	
	
	sprintf(info, "Avg ax: %ld\r\n", sum[0]);
	UART_tx((unsigned char*)info, strlen(info));
	memset(info, 0, 50);
	sprintf(info, "Avg ay: %ld\r\n", sum[1]);
	UART_tx((unsigned char*)info, strlen(info));
	memset(info, 0, 50);
	sprintf(info, "Avg az: %ld\r\n", sum[2]);
	UART_tx((unsigned char*)info, strlen(info));
	memset(info, 0, 50);
	
	//take averages, sign inversion is done when setting the biases
	//note that integer accuracy loss isn't bad because
	//the values are stored as integers on the MPU anyway.
	int16_t accel_bias[3];
	accel_bias[0] = sum[0]; //x
	accel_bias[1] = sum[1]; //y
	accel_bias[2] = sum[2]; //z, from which you must subtract/add 2048 (1g) since z axis measures 1g
	if(accel_bias[2] > 0)
	accel_bias[2] -= 2048; //if z value is positive, subtract 1g;
	else
	accel_bias[2] += 2048; //otherwise, add 1g;
	// note that the if else above does not expect more than 1g of offset (shouldn't be the case)
	
	sprintf(info, "Zeroing bias with: %d\r\n", accel_bias[0]);
	UART_tx((unsigned char*)info, strlen(info));
	memset(info, 0, 50);
	sprintf(info, "Zeroing bias with: %d\r\n", accel_bias[1]);
	UART_tx((unsigned char*)info, strlen(info));
	memset(info, 0, 50);
	sprintf(info, "Zeroing bias with: %d\r\n", accel_bias[2]);
	UART_tx((unsigned char*)info, strlen(info));
	memset(info, 0, 50);
	
	set_accel_bias(accel_bias);
	
	return 0;
}
*/

/****************************************************************************
Reads the factory offsets from the device (only needed for accelerometer
****************************************************************************/
/*
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
*/

/****************************************************************************
This function writes accelerometer biases to the offset registers. The biases
are relative to the current output and are added to the factory values.
LSB in +/- 16g format
****************************************************************************/
/*
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
*/
