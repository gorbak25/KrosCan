/*
 * rfm69.h
 *
 *  	Author: Grzegorz Uriasz, gorbak25@gmail.com
 *
 *		This library is based upon the RFM69 library by Mathis Schmieder (DB9MAT), db9mat@giev.de
 *      This library is heavily based upon the RFM69-STM32 library by André Heßling
 *      	https://github.com/ahessling/RFM69-STM32
 *      Parts were also borrowed from LowPowerLab's RFM69 library for Arduino
 *      	https://github.com/LowPowerLab/RFM69
 *
 *      This library can be used in any non-commercial amateur radio project as long
 *      as the original authors and the source is distributed.
 */

#ifndef RFM69_H_
#define RFM69_H_

#include <stm32f1xx_hal.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <limits.h>

#define RFM69_MAX_FIFO		64 ///< Maximum bytes FIFO

/**
 * Valid RFM69 operation modes.
 */
typedef enum
{
  RFM69_MODE_SLEEP = 0,//!< Sleep mode (lowest power consumption)
  RFM69_MODE_STANDBY,  //!< Standby mode
  RFM69_MODE_FS,       //!< Frequency synthesizer enabled
  RFM69_MODE_TX,       //!< TX mode (carrier active)
  RFM69_MODE_RX        //!< RX mode
} RFM69Mode;

/**
 * Valid RFM69 data modes.
 */
typedef enum
{
  RFM69_DATA_MODE_PACKET = 0,                 //!< Packet engine active
  RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC = 2,   //!< Continuous mode with clock recovery
  RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC = 3,//!< Continuous mode without clock recovery
} RFM69DataMode;

#endif /* RFM69_H_ */

SPI_HandleTypeDef _rfm69_spi;
RFM69Mode _rfm69_mode;
uint8_t _rfm69_init;
uint8_t _rfm69_highPowerDevice;
uint8_t _rfm69_highPowerSettings;
extern SemaphoreHandle_t PayloadReady_sem;
extern SemaphoreHandle_t PacketSent_sem;
extern SemaphoreHandle_t ModeReady_sem;

extern SemaphoreHandle_t Radio_mutex;

extern QueueHandle_t Radio_echo;

extern TaskHandle_t tsk_radio_tx;

//Tasks
void HandleRadioRX();
void HandleRadioTX();

// External functions
uint8_t rfm69_init(SPI_HandleTypeDef spiHandle, int highPowerDevice);
void rfm69_setCustomConfig(const uint8_t config[][2], unsigned int length);
int rfm69_setPowerDBm(int8_t dBm);
int rfm69_send(const void* data, uint32_t dataLength);
//int rfm69_sendBuffer(struct Buffer* buffer);
uint8_t rfm69_receive(char* data);
RFM69Mode rfm69_setMode(RFM69Mode mode);
void rfm69_sleep();
void rfm69_setFrequency(uint32_t frequency);
uint32_t rfm69_getFrequency();
void rfm69_setFrequencyDeviation(unsigned int frequency);
void rfm69_setBitrate(unsigned int bitrate);

// Internal functions
void rfm69_reset(void);
void rfm69_chipSelect(void);
void rfm69_chipUnselect(void);
uint8_t rfm68_readRegister(uint8_t reg);
void rfm69_writeRegister(uint8_t reg, uint8_t value);
void rfm69_setPASettings(uint8_t forcePA);
void rfm69_setPowerLevel(uint8_t power);
int rfm69_setPowerDBm(int8_t dBm);
void rfm69_setHighPowerSettings(uint8_t enable);
void rfm69_waitForModeReady();
void rfm69_waitForPacketSent();
