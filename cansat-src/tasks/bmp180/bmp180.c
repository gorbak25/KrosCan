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

#include "bmp180.h"

void HandleBarometer()
{
	float cur_altitude;
	for(;;)
	{
		cur_altitude = bmp180_readAltitude(BMP180_STANDARD_PRESURE);
		xQueueSendToBack(Barometer_telemetry, (void*)(&cur_altitude), 100);
		vTaskDelay(100);
	}
}

uint8_t bmp180_initialize(uint8_t mode, I2C_HandleTypeDef* i2c) {
  if (mode > BMP180_ULTRAHIGHRES)
    mode = BMP180_ULTRAHIGHRES;
  oversampling = mode;

  bmp180_i2c = i2c;

  if(HAL_I2C_IsDeviceReady(bmp180_i2c, BMP180_I2CADDR, 2, 10) != HAL_OK)
	  return 0;

  if (bmp180_read8(0xD0) != 0x55) return 0;

  /* read calibration data */
  ac1 = bmp180_read16(BMP180_CAL_AC1);
  ac2 = bmp180_read16(BMP180_CAL_AC2);
  ac3 = bmp180_read16(BMP180_CAL_AC3);
  ac4 = bmp180_read16(BMP180_CAL_AC4);
  ac5 = bmp180_read16(BMP180_CAL_AC5);
  ac6 = bmp180_read16(BMP180_CAL_AC6);

  b1 = bmp180_read16(BMP180_CAL_B1);
  b2 = bmp180_read16(BMP180_CAL_B2);

  mb = bmp180_read16(BMP180_CAL_MB);
  mc = bmp180_read16(BMP180_CAL_MC);
  md = bmp180_read16(BMP180_CAL_MD);

  return 1;
}

int32_t bmp180_computeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
  return X1 + X2;
}

uint16_t bmp180_readRawTemperature(void) {
  bmp180_write8(BMP180_CONTROL, BMP180_READTEMPCMD);
  vTaskDelay(5);
  return bmp180_read16(BMP180_TEMPDATA);
}

uint32_t bmp180_readRawPressure(void) {
  uint32_t raw;

  bmp180_write8(BMP180_CONTROL, BMP180_READPRESSURECMD + (oversampling << 6));

  if (oversampling == BMP180_ULTRALOWPOWER)
	vTaskDelay(5);
  else if (oversampling == BMP180_STANDARD)
	vTaskDelay(8);
  else if (oversampling == BMP180_HIGHRES)
	vTaskDelay(14);
  else
	vTaskDelay(26);

  raw = bmp180_read16(BMP180_PRESSUREDATA);

  raw <<= 8;
  raw |= bmp180_read8(BMP180_PRESSUREDATA+2);
  raw >>= (8 - oversampling);

 /* this pull broke stuff, look at it later?
  if (oversampling==0) {
    raw <<= 8;
    raw |= read8(BMP180_PRESSUREDATA+2);
    raw >>= (8 - oversampling);
  }
 */

  return raw;
}

int32_t bmp180_readPressure(void) {
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = bmp180_readRawTemperature();
  UP = bmp180_readRawPressure();

  B5 = bmp180_computeB5(UT);

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  p = p + ((X1 + X2 + (int32_t)3791)>>4);

  return p;
}

int32_t bmp180_readSealevelPressure(float altitude_meters) {
  float pressure = bmp180_readPressure();
  return (int32_t)(pressure / pow(1.0-altitude_meters/44330, 5.255));
}

float bmp180_readTemperature(void) {
  int32_t UT, B5;     // following ds convention
  float temp;

  UT = bmp180_readRawTemperature();

  B5 = bmp180_computeB5(UT);
  temp = (B5+8) >> 4;
  temp /= 10;

  return temp;
}

float bmp180_readAltitude(float sealevelPressure) {
  float altitude;

  float pressure = bmp180_readPressure();

  altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));

  return altitude;
}


/*********************************************************************/

uint8_t bmp180_read8(uint8_t a) {
  uint8_t ret = 0;
  //while ((HAL_SPI_GetState(&bmp180_i2c) != HAL_I2C_STATE_READY));
  HAL_I2C_Mem_Read(bmp180_i2c, BMP180_I2CADDR, a, 1, &ret, 1, HAL_MAX_DELAY);
  return ret;
}

uint16_t bmp180_read16(uint8_t a) {
  uint8_t ret[2];
  HAL_I2C_Mem_Read(bmp180_i2c, BMP180_I2CADDR, a, 1, ret, 2, HAL_MAX_DELAY);
  return (((uint16_t)ret[0])<<8)|((uint16_t)ret[1]);
}

void bmp180_write8(uint8_t a, uint8_t d) {
	HAL_I2C_Mem_Write(bmp180_i2c, BMP180_I2CADDR, a, 1, &d, 1, HAL_MAX_DELAY);
}
