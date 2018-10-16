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
#define ACCELEROMETER_RES 0.1220703125 // mg/LSB
#define GYROSCOPE_RES 0.0304878049 // (�/s)/LSB
#define MAGNETOMETER_RES 1.5 // G/LSB
#define PI 3.14159265358979323846
#define deg_to_rad(deg) ((float)deg * PI / 180.0)

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

static float hard_iron_correction[3] = {0, 0, 0}; //x, y, z hard iron corrections, subtract from scaled measurements
static float soft_iron_correction[3] = {0, 0, 0}; //x, y, z soft iron corrections, multiply with scaled and hard corrected measurements
// also see https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
static int8_t magneto_sens_adjust[3] = {0, 0, 0}; // x, y, z sens adjust values
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
uint8_t get_magnetometer_scale();
uint8_t read_magnetometer(int16_t*);
uint8_t scale_magnetometer(float*);
uint8_t set_accelerometer_4g();
uint8_t read_accelerometer_data(int16_t*);
uint8_t read_gyroscope_data(int16_t*);
uint8_t apply_hard_iron_correction(float*);
uint8_t apply_soft_iron_correction(float*);

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
	
	// enable global interrupts such that the I2C and UART interfaces can operate
	sei();
	
	// Initialise and calibrate MPU9250;
	MPU_9250_initialise();
	calibrate_accelerometer;
	calibrate_gyroscope();
	// put some code here so you know when to do the 8 pattern.
	get_magnetometer_scale();
	calibrate_magnetometer();
	
	//gyro is by now set to 1000 dps, which is ok
	//magneto can be set to +/- 4g mode
	set_accelerometer_4g();
	
	// Init Bluetooth controller (if needed)
	float magnetometer_data_gaus[3];
	float gyroscope_data_dps[3];
	float accelerometer_data_g[3];
	int16_t magnetometer_data_raw[3];
	int16_t gyroscope_data_raw[3];
	int16_t accelerometer_data_raw[3];
	float roll = 0;
	float yaw = 0;
	float pitch = 0;
	Mahony mahony;
	mahony.begin(100);//100 Hz is the most we can do due to magnetometer speed
	
    while (1) 
    {
		/* obtain measurements and feed Mahony filter. */
		read_gyroscope_data(gyroscope_data_raw);
		read_accelerometer_data(accelerometer_data_raw);
		read_magnetometer(magnetometer_data_raw);
		// magnetometer has to be converted to float earlier.
		magnetometer_data_gaus[0] = (float) magnetometer_data_raw[0];
		magnetometer_data_gaus[1] = (float) magnetometer_data_raw[1];
		magnetometer_data_gaus[2] = (float) magnetometer_data_raw[2];
		scale_magnetometer(magnetometer_data_gaus);
		apply_hard_iron_correction(magnetometer_data_gaus);
		apply_soft_iron_correction(magnetometer_data_gaus);
		
		//conversion to appropriate datatypes happens here
		for(uint8_t i; i < 3; ++i){
			gyroscope_data_dps[i] = (float)(GYROSCOPE_RES * gyroscope_data_raw[i]);
			accelerometer_data_g[i] = (float)(ACCELEROMETER_RES * accelerometer_data_raw[i]);
			magnetometer_data_gaus[i] = (float)(MAGNETOMETER_RES * magnetometer_data_gaus[i]);
		}
		
		mahony.update(	deg_to_rad(gyroscope_data_dps[0]),
						deg_to_rad(gyroscope_data_dps[1]),
						deg_to_rad(gyroscope_data_dps[2]),
						accelerometer_data_g[0],
						accelerometer_data_g[1],
						accelerometer_data_g[2],
						magnetometer_data_gaus[0],
						magnetometer_data_gaus[1],
						magnetometer_data_gaus[2]);
		
		roll = mahony.getRoll();
		yaw = mahony.getYaw();
		pitch = mahony.getPitch();
		/* update Bluetooth controller with axes info */
    }
}

/****************************************************************************
Call this function to initialise the MPU_9250, just selects clock src.
****************************************************************************/
uint8_t MPU_9250_initialise(){
	//configure clock source
	uint8_t write_cmd[3];
	write_cmd[0] = MPU_9250_Addr << 1; // write
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
uint8_t calibrate_accelerometer(){
	// Set accelerometer range to +/-16g since offset is stored in that format
	uint8_t write_cmd[3];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 28; // Accelerometer configuration register
	write_cmd[2] = 0b00011000; // set ACCEL_FS_SEL to 11 or +/- 16g;
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	
	// take 1024 samples to average
	
	int32_t sum[3];
	int16_t accelerometer_data[3];
	for(uint16_t i = 0; i < 1024; ++i){
		read_accelerometer_data(accelerometer_data);
		
		//accumulate immediately to limit RAM usage
		sum[0] += accelerometer_data[0];
		sum[1] += accelerometer_data[1];
		sum[2] += accelerometer_data[2];
		
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
Reads accelerometer data, assumes data is being collected continuously
****************************************************************************/
uint8_t read_accelerometer_data(int16_t* accel_data){
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
	return 0;
}

/****************************************************************************
Sets accelerometer in 4g range
****************************************************************************/
uint8_t set_accelerometer_4g(){
	// Set accelerometer range to +/-16g since offset is stored in that format
	uint8_t write_cmd[3];
	write_cmd[0] = MPU_9250_Addr << 1; // write
	write_cmd[1] = 28; // Accelerometer configuration register
	write_cmd[2] = 0b00001000; // set ACCEL_FS_SEL to 01 or +/- 4g;
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
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
	
	int32_t sum[3];
	int16_t gyro_data[3];
	// take 1024 samples to average
	for(uint16_t i = 0; i < 1024; ++i){
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
	gyro_bias[0] = (int16_t)(sum[0]/1024); //x
	gyro_bias[1] = (int16_t)(sum[1]/1024); //y
	gyro_bias[2] = (int16_t)(sum[2]/1024); //z
	
	set_gyro_bias(gyro_bias);
	
	return 0;
}

/****************************************************************************
This function reads gyroscope data.
It assumes data is being sampled
****************************************************************************/
uint8_t read_gyroscope_data(int16_t* gyro_data){
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

/****************************************************************************
This function gets the hard and soft iron offsets used to correct magnetometer
values.
It assumes the magneto is in power down mode at the start.
****************************************************************************/
uint8_t calibrate_magnetometer(){
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
	for(uint8_t i = 0; i < 1024; ++i){
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
	int16_t hard_iron_raw[3]; // int16 suffices normally
	hard_iron_raw[0] = (max[0] + min[0])/2;
	hard_iron_raw[1] = (max[1] + min[1])/2;
	hard_iron_raw[2] = (max[2] + min[2])/2;
	
	//scale these using asa adjustment values, convert to float first
	hard_iron_correction[0] = (float)hard_iron_raw[0];
	hard_iron_correction[1] = (float)hard_iron_raw[1];
	hard_iron_correction[2] = (float)hard_iron_raw[2];
	scale_magnetometer(hard_iron_correction);
	// hard iron correction is now done.
	
	// calculate soft iron correction (at once in float since I must scale them afterwards)
	float soft_iron_raw[3];
	soft_iron_raw[0] = (float)((max[0] - min[0])/2);
	soft_iron_raw[1] = (float)((max[1] - min[1])/2);
	soft_iron_raw[2] = (float)((max[2] - min[2])/2);
	
	// scale soft iron
	float* soft_iron_scaled = soft_iron_raw;
	scale_magnetometer(soft_iron_scaled);
	
	float avg_rad = soft_iron_scaled[0] + soft_iron_scaled[1] + soft_iron_scaled[2];
	avg_rad /= 3.0;
	
	soft_iron_correction[0] = avg_rad/soft_iron_scaled[0];
	soft_iron_correction[1] = avg_rad/soft_iron_scaled[1];
	soft_iron_correction[2] = avg_rad/soft_iron_scaled[2];
	
	// There is no need to power down the magnetometer again, because hereafter
	// the normal program flow will commence.
	return 0;
}

/****************************************************************************
This function scales magnetometer readings with the scale adjustment values.
scaling happens in place
****************************************************************************/
uint8_t scale_magnetometer(float* magneto_data){
	magneto_data[0] = magneto_data[0] * ((float)magneto_sens_adjust[0] * 0.5 / 128.0 + 1.0);
	magneto_data[1] = magneto_data[1] * ((float)magneto_sens_adjust[1] * 0.5 / 128.0 + 1.0);
	magneto_data[2] = magneto_data[2] * ((float)magneto_sens_adjust[2] * 0.5 / 128.0 + 1.0);
	
	return 0;
}

/****************************************************************************
This function reads the magnetometer data. parameter is empty array of x, y, z
The function assumes the magnetometer is in a measurement mode
****************************************************************************/
uint8_t read_magnetometer(int16_t* magneto_data){
	
	uint8_t write_cmd[2];
	write_cmd[0] = AK8963_addr << 1; //Write
	write_cmd[1] = 0x03; // HXL
	uint8_t read_cmd[7];
	read_cmd[0] = AK8963_addr << 1 | 1; // Read
	TWI_Start_Transceiver_With_Data(write_cmd, 2);
	TWI_Start_Transceiver_With_Data(read_cmd, 7);
	TWI_Get_Data_From_Transceiver(read_cmd, 7);
	magneto_data[0] = read_cmd[2] << 8 | read_cmd[1];
	magneto_data[1] = read_cmd[4] << 8 | read_cmd[3];
	magneto_data[2] = read_cmd[6] << 8 | read_cmd[5];
	
	return 0;
}

/****************************************************************************
This function gets the magnetometer scale adjustment, used later on in the
program.
It assumes the magneto is in power down mode at the start.
****************************************************************************/
uint8_t get_magnetometer_scale(){
	//First set magnetometer in Fuse ROM Access mode
	uint8_t write_cmd[3];
	write_cmd[0] = AK8963_addr << 1; //Write
	write_cmd[1] = 0x0A; // CNTL1
	write_cmd[2] = 0x0F; // Fuse ROM Access Mode
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	//The device is now in Fuse ROM Access Mode
	//Read the magneto sens adjust (int8)
	memset(write_cmd, 3, 0);
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
	memset(write_cmd, 3, 0);
	write_cmd[0] = AK8963_addr << 1; //Write
	write_cmd[1] = 0x0A; // CNTL1
	write_cmd[2] = 0x00; // Power Down Mode
	TWI_Start_Transceiver_With_Data(write_cmd, 3);
	// datasheet asks to wait 100 �s before transitioning to other modes,
	// delay here to be sure this is respected
	_delay_us(100);
	
	return 0;
}

uint8_t apply_hard_iron_correction(float* magneto_data){
	magneto_data[0] = magneto_data[0] - hard_iron_correction[0];
	magneto_data[1] = magneto_data[1] - hard_iron_correction[1];
	magneto_data[2] = magneto_data[2] - hard_iron_correction[2];
	return 0;
}

uint8_t apply_soft_iron_correction(float* magneto_data){
	magneto_data[0] = magneto_data[0] * soft_iron_correction[0];
	magneto_data[1] = magneto_data[1] * soft_iron_correction[1];
	magneto_data[2] = magneto_data[2] * soft_iron_correction[2];
	return 0;
}
