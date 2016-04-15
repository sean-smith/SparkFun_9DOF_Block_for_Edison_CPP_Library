/******************************************************************************
SFE_LSM9DS0.cpp
SFE_LSM9DS0 Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 14, 2014 (Happy Valentines Day!)
Modified 14 Jul 2015 by Mike Hord to add Edison support
https://github.com/sparkfun/SparkFun_9DOF_Block_for_Edison_CPP_Library

This file implements all functions of the LSM9DS0 class. Functions here range
from higher level stuff, like reading/writing LSM9DS0 registers to low-level,
hardware reads and writes.

** Supports only I2C connections!!! **

Development environment specifics:
  Code developed in Intel's Eclipse IOT-DK
  This code requires the Intel mraa library to function; for more
  information see https://github.com/intel-iot-devkit/mraa

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SFE_LSM9DS0.h"

// Setup the imu
LSM9DS0_t* imu_setup(uint8_t gAddr, uint8_t xmAddr)
{
	LSM9DS0_t* imu;

	imu->gx = 0;
	imu->gy = 0;
	imu->gz = 0;

	imu->ax = 0;
	imu->ay = 0;
	imu->az = 0;

	imu->mx = 0;
	imu->my = 0;
	imu->mz = 0;

	imu->temperature = 0;

	imu->gScale = G_SCALE_245DPS;
	imu->aScale = A_SCALE_4G;
	imu->mScale = M_SCALE_2GS;

	imu->gRes = 0;
	imu->aRes = 0;
	imu->mRes = 0;

	// setting up the i2c
	imu->gyro = mraa_i2c_init(1);
	mraa_i2c_address(imu->gyro, gAddr);
	imu->xm = mraa_i2c_init(1);
	mraa_i2c_address(imu->xm, xmAddr);

	return imu;
}

uint16_t begin(LSM9DS0_t* imu, gyro_scale gScl, accel_scale aScl,
		mag_scale mScl, gyro_odr gODR, accel_odr aODR, mag_odr mODR)
{
	// Store the given scales in imu struct. These scale variables
	// are used throughout to calculate the actual g's, DPS,and Gs's.
	imu->gScale = gScl;
	imu->aScale = aScl;
	imu->mScale = mScl;

	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(imu); // Calculate DPS / ADC tick, stored in gRes variable
	calcaRes(imu); // Calculate g / ADC tick, stored in aRes variable
	calcmRes(imu); // Calculate Gs / ADC tick, stored in mRes variable

	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t gTest = gReadByte(imu->gyro, WHO_AM_I_G);		// Read the gyro WHO_AM_I
	uint8_t xmTest = xmReadByte(imu->xm, WHO_AM_I_XM);	// Read the accel/mag WHO_AM_I
	
	// Gyro initialization stuff:
	initGyro(imu->gyro);	// This will "turn on" the gyro. Setting up interrupts, etc.
	setGyroODR(imu->gyro, gODR); // Set the gyro output data rate and bandwidth.
	setGyroScale(imu, imu->gScale); // Set the gyro range
}

void initGyro(mraa_i2c_context gyro)
{
	/* CTRL_REG1_G sets output data rate, bandwidth, power-down and enables
	Bits[7:0]: DR1 DR0 BW1 BW0 PD Zen Xen Yen
	DR[1:0] - Output data rate selection
		00=95Hz, 01=190Hz, 10=380Hz, 11=760Hz
	BW[1:0] - Bandwidth selection (sets cutoff frequency)
		 Value depends on ODR. See datasheet table 21.
	PD - Power down enable (0=power down mode, 1=normal or sleep mode)
	Zen, Xen, Yen - Axis enable (o=disabled, 1=enabled)	*/
	gWriteByte(gyro, CTRL_REG1_G, 0x0F); // Normal mode, enable all axes
	
	/* CTRL_REG2_G sets up the HPF
	Bits[7:0]: 0 0 HPM1 HPM0 HPCF3 HPCF2 HPCF1 HPCF0
	HPM[1:0] - High pass filter mode selection
		00=normal (reset reading HP_RESET_FILTER, 01=ref signal for filtering,
		10=normal, 11=autoreset on interrupt
	HPCF[3:0] - High pass filter cutoff frequency
		Value depends on data rate. See datasheet table 26.
	*/
	gWriteByte(gyro, CTRL_REG2_G, 0x00); // Normal mode, high cutoff frequency
	
	/* CTRL_REG3_G sets up interrupt and DRDY_G pins
	Bits[7:0]: I1_IINT1 I1_BOOT H_LACTIVE PP_OD I2_DRDY I2_WTM I2_ORUN I2_EMPTY
	I1_INT1 - Interrupt enable on INT_G pin (0=disable, 1=enable)
	I1_BOOT - Boot status available on INT_G (0=disable, 1=enable)
	H_LACTIVE - Interrupt active configuration on INT_G (0:high, 1:low)
	PP_OD - Push-pull/open-drain (0=push-pull, 1=open-drain)
	I2_DRDY - Data ready on DRDY_G (0=disable, 1=enable)
	I2_WTM - FIFO watermark interrupt on DRDY_G (0=disable 1=enable)
	I2_ORUN - FIFO overrun interrupt on DRDY_G (0=disable 1=enable)
	I2_EMPTY - FIFO empty interrupt on DRDY_G (0=disable 1=enable) */
	// Int1 enabled (pp, active low), data read on DRDY_G:
	gWriteByte(gyro, CTRL_REG3_G, 0x88); 
	
	/* CTRL_REG4_G sets the scale, update mode
	Bits[7:0] - BDU BLE FS1 FS0 - ST1 ST0 SIM
	BDU - Block data update (0=continuous, 1=output not updated until read
	BLE - Big/little endian (0=data LSB @ lower address, 1=LSB @ higher add)
	FS[1:0] - Full-scale selection
		00=245dps, 01=500dps, 10=2000dps, 11=2000dps
	ST[1:0] - Self-test enable
		00=disabled, 01=st 0 (x+, y-, z-), 10=undefined, 11=st 1 (x-, y+, z+)
	SIM - SPI serial interface mode select
		0=4 wire, 1=3 wire */
	gWriteByte(gyro, CTRL_REG4_G, 0x00); // Set scale to 245 dps
	
	/* CTRL_REG5_G sets up the FIFO, HPF, and INT1
	Bits[7:0] - BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
	BOOT - Reboot memory content (0=normal, 1=reboot)
	FIFO_EN - FIFO enable (0=disable, 1=enable)
	HPen - HPF enable (0=disable, 1=enable)
	INT1_Sel[1:0] - Int 1 selection configuration
	Out_Sel[1:0] - Out selection configuration */
	gWriteByte(gyro, CTRL_REG5_G, 0x00);
	
}

void setGyroScale(LSM9DS0_t* imu, gyro_scale gScl)
{
	// We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
	uint8_t temp = gReadByte(imu->gyro, CTRL_REG4_G);
	// Then mask out the gyro scale bits:
	temp &= 0xFF^(0x3 << 4);
	// Then shift in our new scale bits:
	temp |= gScl << 4;
	// And write the new register value back into CTRL_REG4_G:
	gWriteByte(imu->gyro, CTRL_REG4_G, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update gScale:
	imu->gScale = gScl;
	// Then calculate a new gRes, which relies on gScale being set correctly:
	calcgRes(imu);
}

void setGyroODR(mraa_i2c_context gyro, gyro_odr gRate)
{
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = gReadByte(gyro, CTRL_REG1_G);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (gRate << 4);
	// And write the new register value back into CTRL_REG1_G:
	gWriteByte(gyro, CTRL_REG1_G, temp);
}

void calcgRes(LSM9DS0_t* imu)
{
	// Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
	// to calculate DPS/(ADC tick) based on that 2-bit value:
	switch (imu->gScale)
	{
	case G_SCALE_245DPS:
		imu->gRes = 245.0 / 32768.0;
		break;
	case G_SCALE_500DPS:
		imu->gRes = 500.0 / 32768.0;
		break;
	case G_SCALE_2000DPS:
		imu->gRes = 2000.0 / 32768.0;
		break;
	}
}


void calcaRes(LSM9DS0_t* imu)
{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an 
	// algorithm to calculate g/(ADC tick) based on that 3-bit value:
	imu->aRes = imu->aScale == A_SCALE_16G ? 16.0 / 32768.0 : 
		   (((float) imu->aScale + 1.0) * 2.0) / 32768.0;
}

void calcmRes(LSM9DS0_t* imu)
{
	// Possible magnetometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11). Here's a bit of an algorithm
	// to calculate Gs/(ADC tick) based on that 2-bit value:
	imu->mRes = imu->mScale == M_SCALE_2GS ? 2.0 / 32768.0 : 
	       (float) (imu->mScale << 2) / 32768.0;
}

void gWriteByte(mraa_i2c_context gyro, uint8_t subAddress, uint8_t data)
{
	mraa_i2c_write_byte_data(gyro, data, subAddress);
}

void xmWriteByte(mraa_i2c_context xm, uint8_t subAddress, uint8_t data)
{
	mraa_i2c_write_byte_data(xm, data, subAddress);
}

uint8_t gReadByte(mraa_i2c_context gyro, uint8_t subAddress)
{
  return mraa_i2c_read_byte_data(gyro, subAddress);
}


uint8_t xmReadByte(mraa_i2c_context xm, uint8_t subAddress)
{
  return mraa_i2c_read_byte_data(xm, subAddress);
}
