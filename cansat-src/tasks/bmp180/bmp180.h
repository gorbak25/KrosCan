/***************************************************
  This is a library for the Adafruit BMP085/BMP180 Barometric Pressure + Temp sensor

  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603

  These displays use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/*
 * Library modified for FreeRTOS by Grzegorz Uriasz, gorbak25@gmail.com
 */

#include <stm32f1xx_hal.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <limits.h>
#include <math.h>

#ifndef BMP180_H
#define BMP180_H

#define BMP180_STANDARD_PRESURE 101325
#define BMP180_I2CADDR 0xEE

/**
 * Valid BMP180 operation modes.
 */
typedef enum
{
	BMP180_ULTRALOWPOWER = 0, //highest sampling rate lowest accuracy
	BMP180_STANDARD = 1,
	BMP180_HIGHRES = 2,
	BMP180_ULTRAHIGHRES = 3   //lowest sampling rate highest accuracy
} BMP180Mode;


I2C_HandleTypeDef* bmp180_i2c;
BMP180Mode oversampling;
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;

#define BMP180_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP180_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP180_CAL_AC3           0xAE  // R   Calibration data (16 bits)
#define BMP180_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP180_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP180_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP180_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP180_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP180_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP180_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP180_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP180_CONTROL           0xF4
#define BMP180_TEMPDATA          0xF6
#define BMP180_PRESSUREDATA      0xF6
#define BMP180_READTEMPCMD          0x2E
#define BMP180_READPRESSURECMD            0x34

//Tasks
void HandleBarometer();

//Functions
uint8_t bmp180_initialize(BMP180Mode mode, I2C_HandleTypeDef* i2c);  // by default go highres
float bmp180_readTemperature(void);
int32_t bmp180_readPressure(void);
int32_t bmp180_readSealevelPressure(float altitude_meters);
float bmp180_readAltitude(float sealevelPressure); // std atmosphere = 101325
uint16_t bmp180_readRawTemperature(void);
uint32_t bmp180_readRawPressure(void);

int32_t bmp180_computeB5(int32_t UT);
uint8_t bmp180_read8(uint8_t addr);
uint16_t bmp180_read16(uint8_t addr);
void bmp180_write8(uint8_t addr, uint8_t data);

#endif //  BMP180_H
