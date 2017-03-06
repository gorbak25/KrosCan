/*
 * rfm69.c
 *
 *  Created on: Nov 26, 2016
 *      Author: Mathis Schmieder (DB9MAT), db9mat@giev.de
 *
 *      This library is heavily based upon the RFM69-STM32 library by André Heßling
 *      	https://github.com/ahessling/RFM69-STM32
 *      Parts were also borrowed from LowPowerLab's RFM69 library for Arduino
 *      	https://github.com/LowPowerLab/RFM69
 *
 *      This library can be used in any non-commercial amateur radio project as long
 *      as the original authors and the source is distributed.
 */

#include "radio.h"
#include <task.h>
#include "RFM69registers.h"

#define TIMEOUT_MODE_READY    100 ///< Maximum amount of time until mode switch [ms]
#define TIMEOUT_PACKET_SENT   1000 ///< Maximum amount of time until packet must be sent [ms]
#define CSMA_LIMIT              -70 // upper RX signal sensitivity threshold in dBm for carrier sense access

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static BaseType_t xHigherPriorityTaskWoken;

	if(HAL_GPIO_ReadPin(RADIO_INT_GPIO_Port, RADIO_INT_Pin) == GPIO_PIN_SET)
	{
		//unlock RX thread
		if(_rfm69_mode == RFM69_MODE_RX)
		{
			xSemaphoreGiveFromISR(PayloadReady_sem, &xHigherPriorityTaskWoken);
		}
		else if(_rfm69_mode == RFM69_MODE_TX)
		{
			xSemaphoreGiveFromISR(PacketSent_sem, &xHigherPriorityTaskWoken);
		}
	}
	else if(HAL_GPIO_ReadPin(RADIO_MODEREADY_GPIO_Port, RADIO_MODEREADY_Pin) == GPIO_PIN_SET)
	{
		xSemaphoreGiveFromISR(ModeReady_sem, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

char bufrx[50];
void HandleRadioRX()
{
	for(;;)
	{
		if(xSemaphoreTake(PayloadReady_sem, portMAX_DELAY)) //wait for payload ready
		{
			//claim the radio for us
			if(xSemaphoreTake(Radio_mutex, portMAX_DELAY))
			{
				uint8_t len = rfm69_receive(bufrx);
				int rssi = rfm69_readRSSI();

				// switch to RX mode
				rfm69_setMode(RFM69_MODE_RX);

				xSemaphoreGive(Radio_mutex);

				if(len>0)
				{
					//payload ready so exit the csma loop
					xTaskNotifyGive(tsk_radio_tx);
					taskYIELD(); //if tx is waiting

					/*taskENTER_CRITICAL();
					trace_printf("%s\n", bufrx);
					taskEXIT_CRITICAL();*/

					//taskENTER_CRITICAL();
					char lenc = (char)len;
					xQueueSendToBack(Radio_echo, &lenc, 0);
					for(uint8_t i = 0; i<len; i++)
					{
						xQueueSendToBack(Radio_echo, (void*)(bufrx+i), 0);
					}
					//taskEXIT_CRITICAL();
				}

				/*__disable_irq();
				trace_printf("asd");
				__enable_irq();*/
			}
		}
	}
}

char buftx[50];
int buftx_pos;
uint8_t buftx_full;
const uint8_t max_packet_len = 50;
void HandleRadioTX()
{
	buftx_pos = 0;
	buftx_full = 0;
	for(;;)
	{
		//if no more data can be buffered then send out the packet
		if(buftx_full == 1)
		{
			rfm69_send(buftx, buftx_pos);
			buftx_full = 0;
			buftx_pos = 0;
		}

		if(uxQueueMessagesWaiting(Radio_echo)>0)
		{
			char msg_len;
			xQueuePeek(Radio_echo, &msg_len, 0);

			if(msg_len+buftx_pos+2 > max_packet_len)
			{
				buftx_full = 1; //send the packet
			}
			else
			{
				buftx[buftx_pos] = 'E'; //echo ctr val
				buftx_pos++;
				buftx[buftx_pos] = msg_len; //echo ctr val
				buftx_pos++;
				xQueueReceive(Radio_echo, &msg_len, 0);

				for(uint8_t i = 0; i<msg_len; i++)
				{
					xQueueReceive(Radio_echo, buftx+buftx_pos, 1);
					buftx_pos++;
				}
			}
		}
		else if(uxQueueMessagesWaiting(Barometer_telemetry)>0)
		{
			if(1+sizeof(BMP180TelemetryData)+buftx_pos > max_packet_len)
			{
				buftx_full = 1; //send the packet
			}
			else
			{
				buftx[buftx_pos] = 'A';
				xQueueReceive(Barometer_telemetry, buftx+buftx_pos+1, 0);
				buftx_pos += sizeof(BMP180TelemetryData)+1;
			}
		}
		else
		{
			vTaskDelay(10); //wait for new data
		}

	}
}


/** RFM69 base configuration after init().
 *
 * Change these to your needs or call rfm69_setCustomConfig() after module init.
 */
static const uint8_t rfm69_base_config[][2] =
{
/*	    {0x01, 0x04}, // RegOpMode: Standby Mode
	    {0x02, 0x00}, // RegDataModul: Packet mode, FSK, no shaping
	    {0x03, 0x68}, // RegBitrateMsb: 1.2 kbps
	    {0x04, 0x2B}, // RegBitrateLsb
	    {0x05, 0x00}, // RegFdevMsb: 4.5 kHz
	    {0x06, 0x4a}, // RegFdevLsb
	    {0x07, 0xD9}, // RegFrfMsb: 868,15 MHz // TODO
	    {0x08, 0x09}, // RegFrfMid
	    {0x09, 0x9A}, // RegFrfLsb
	    {0x18, 0x88}, // RegLNA: 200 Ohm impedance, gain set by AGC loop
	    {0x19, 0x4C}, // RegRxBw: 25 kHz
	    {0x2C, 0x00}, // RegPreambleMsb: disable preamble
	    {0x2D, 0x72}, // RegPreambleLsb
	    {0x2E, 0x00}, // RegSyncConfig: Disable sync word
	    {0x2F, 0x00}, // RegSyncValue1
	    {0x30, 0x00}, // RegSyncValue2
	    {0x37, 0x00}, // RegPacketConfig1: Fixed length, CRC off, no whitening
	    {0x38, 0x00}, // RegPayloadLength: Unlimited payload
	    {0x3C, 0x8F}, // RegFifoThresh: TxStart on FifoNotEmpty, 15 bytes FifoLevel
	    {0x58, 0x1B}, // RegTestLna: Normal sensitivity mode
	    {0x6F, 0x30}, // RegTestDagc: Improved margin, use if AfcLowBetaOn=0 (default) */
	    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
	    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
	    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_9600}, // default: 4.8 KBPS
	    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_9600},
	    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
	    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

	    /* 0x07 */ { REG_FRFMSB, RF_FRFMSB_433 },
	    /* 0x08 */ { REG_FRFMID, RF_FRFMID_433 },
	    /* 0x09 */ { REG_FRFLSB, RF_FRFLSB_433 },

	    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
	    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
	    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
	    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00 }, // DIO0 is the only IRQ we're using
	    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF | RF_DIOMAPPING2_DIO5_11 }, // DIO5 ClkOut disable for power saving DIO5 Mode Ready
	    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
	    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
	    ///* 0x2D */ { REG_PREAMBLELSB, 0x00}, // 144 preamble bytes 0xAAAAAA
	    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
	    /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
	    /* 0x30 */ { REG_SYNCVALUE2, 0x00 }, // NETWORK ID
	    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
	    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // 0: unlimited packet format
	    ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
	    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
	    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
	    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
	    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
	    {255, 0}
};

// Clock constants. DO NOT CHANGE THESE!
#define RFM69_XO               32000000    ///< Internal clock frequency [Hz]
#define RFM69_FSTEP            61.03515625 ///< Step width of synthesizer [Hz]

void rfm69_reset()
{
	// generate reset impulse
	HAL_GPIO_WritePin(RADIO_RESET_GPIO_Port, RADIO_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(RADIO_RESET_GPIO_Port, RADIO_RESET_Pin, GPIO_PIN_RESET);

	// wait until module is ready
	HAL_Delay(10);

	_rfm69_init = 0;
	_rfm69_mode = RFM69_MODE_STANDBY;
}

void rfm69_chipSelect()
{
	HAL_GPIO_WritePin(RADIO_SS_GPIO_Port, RADIO_SS_Pin, GPIO_PIN_RESET);
}

void rfm69_chipUnselect()
{
	HAL_GPIO_WritePin(RADIO_SS_GPIO_Port, RADIO_SS_Pin, GPIO_PIN_SET);
}

/**
 * Read a RFM69 register value.
 *
 * @param reg The register to be read
 * @return The value of the register
 */
uint8_t rfm69_readRegister(uint8_t reg)
{
  // sanity check
  if (reg > 0x7f)
    return 0;

  // read value from register
  rfm69_chipSelect();

  uint8_t value;
  HAL_SPI_Transmit(&_rfm69_spi, &reg , 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&_rfm69_spi, &value, 1, HAL_MAX_DELAY);

  rfm69_chipUnselect();

  return value;
}

/**
 * Write a RFM69 register value.
 *
 * @param reg The register to be written
 * @param value The value of the register to be set
 */
void rfm69_writeRegister(uint8_t reg, uint8_t value)
{
  // sanity check
  if (reg > 0x7f)
    return;

  // transfer value to register and set the write flag
  rfm69_chipSelect();

  uint8_t wreg = (reg | 0x80);
  HAL_SPI_Transmit(&_rfm69_spi, &wreg , 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&_rfm69_spi, &value, 1, HAL_MAX_DELAY);

  rfm69_chipUnselect();
}

/**
 * Reconfigure the RFM69 module by writing multiple registers at once.
 *
 * @param config Array of register/value tuples
 * @param length Number of elements in config array
 */
void rfm69_setCustomConfig(const uint8_t config[][2], unsigned int length)
{
  for (unsigned int i = 0; i < length; i++)
  {
    rfm69_writeRegister(config[i][0], config[i][1]);
  }
}

/**
 * Initialize the RFM69 module.
 * A base configuration is set and the module is put in standby mode.
 *
 * @return Always true
 */
uint8_t rfm69_init(SPI_HandleTypeDef spi, int highPowerDevice)
{
  _rfm69_spi = spi;
  _rfm69_highPowerDevice = highPowerDevice;

  // reset chip
  rfm69_reset();

  // set base configuration
  rfm69_setCustomConfig(rfm69_base_config, sizeof(rfm69_base_config) / 2);

  // set PA and OCP settings according to RF module (normal/high power)
  rfm69_setPASettings(0);

  // disable encryption. this is amateur radio
  rfm69_writeRegister(0x3D, (rfm69_readRegister(0x3D) & 0xFE) | 0);

  rfm69_setMode(RFM69_MODE_STANDBY);

  _rfm69_init = 1;

  return _rfm69_init;
}

/**
 * Enable/disable the power amplifier(s) of the RFM69 module.
 *
 * PA0 for regular devices is enabled and PA1 is used for high power devices (default).
 *
 * @note Use this function if you want to manually override the PA settings.
 * @note PA0 can only be used with regular devices (not the high power ones!)
 * @note PA1 and PA2 can only be used with high power devices (not the regular ones!)
 *
 * @param forcePA If this is 0, default values are used. Otherwise, PA settings are forced.
 *                0x01 for PA0, 0x02 for PA1, 0x04 for PA2, 0x08 for +20 dBm high power settings.
 */
void rfm69_setPASettings(uint8_t forcePA)
{
  // disable OCP for high power devices, enable otherwise
  rfm69_writeRegister(0x13, 0x0A | (_rfm69_highPowerDevice ? 0x00 : 0x10));

  if (0 == forcePA)
  {
    if (_rfm69_highPowerDevice > 0)
    {
      // enable PA1 only
      rfm69_writeRegister(0x11, (rfm69_readRegister(0x11) & 0x1F) | 0x40);
    }
    else
    {
      // enable PA0 only
      rfm69_writeRegister(0x11, (rfm69_readRegister(0x11) & 0x1F) | 0x80);
    }
  }
  else
  {
    // PA settings forced
    uint8_t pa = 0;

    if (forcePA & 0x01)
      pa |= 0x80;

    if (forcePA & 0x02)
      pa |= 0x40;

    if (forcePA & 0x04)
      pa |= 0x20;

    // check if high power settings are forced
    _rfm69_highPowerSettings = (forcePA & 0x08) ? 1 : 0;
    rfm69_setHighPowerSettings(_rfm69_highPowerSettings);

    rfm69_writeRegister(0x11, (rfm69_readRegister(0x11) & 0x1F) | pa);
  }
}

/**
 * Set the output power level of the RFM69 module.
 *
 * @param power Power level from 0 to 31.
 */
void rfm69_setPowerLevel(uint8_t power)
{
  if (power > 31)
    power = 31;

  rfm69_writeRegister(0x11, (rfm69_readRegister(0x11) & 0xE0) | power);
}

/**
 * Enable the +20 dBm high power settings of RFM69Hxx modules.
 *
 * @note Enabling only works with high power devices.
 *
 * @param enable true or false
 */
void rfm69_setHighPowerSettings(uint8_t enable)
{
  // enabling only works if this is a high power device
  if (enable > 0 && _rfm69_highPowerDevice == 0)
    enable = 0;

  rfm69_writeRegister(0x5A, enable ? 0x5D : 0x55);
  rfm69_writeRegister(0x5C, enable ? 0x7C : 0x70);
}

int rfm69_setPowerDBm(int8_t dBm)
{
  /* Output power of module is from -18 dBm to +13 dBm
   * in "low" power devices, -2 dBm to +20 dBm in high power devices */
  if (dBm < -18 || dBm > 20)
    return -1;

  if (_rfm69_highPowerDevice == 0 && dBm > 13)
    return -1;

  if (_rfm69_highPowerDevice == 0 && dBm < -2)
    return -1;

  uint8_t powerLevel = 0;

  if (_rfm69_highPowerDevice == 0)
  {
    // only PA0 can be used
    powerLevel = dBm + 18;

    // enable PA0 only
    rfm69_writeRegister(0x11, 0x80 | powerLevel);
  }
  else
  {
    if (dBm >= -2 && dBm <= 13)
    {
      // use PA1 on pin PA_BOOST
      powerLevel = dBm + 18;

      // enable PA1 only
      rfm69_writeRegister(0x11, 0x40 | powerLevel);

      // disable high power settings
      _rfm69_highPowerSettings = 0;
      rfm69_setHighPowerSettings(_rfm69_highPowerSettings);
    }
    else if (dBm > 13 && dBm <= 17)
    {
      // use PA1 and PA2 combined on pin PA_BOOST
      powerLevel = dBm + 14;

      // enable PA1+PA2
      rfm69_writeRegister(0x11, 0x60 | powerLevel);

      // disable high power settings
      _rfm69_highPowerSettings = 0;
      rfm69_setHighPowerSettings(_rfm69_highPowerSettings);
    }
    else
    {
      // output power from 18 dBm to 20 dBm, use PA1+PA2 with high power settings
      powerLevel = dBm + 11;

      // enable PA1+PA2
      rfm69_writeRegister(0x11, 0x60 | powerLevel);

      // enable high power settings
      _rfm69_highPowerSettings = 1;
      rfm69_setHighPowerSettings(_rfm69_highPowerSettings);
    }
  }

  return 0;
}

int rfm69_readRSSI() {
	int _rssi = -rfm69_readRegister(0x24) / 2;

	/*taskENTER_CRITICAL();
	trace_printf("%d\n", _rssi);
	taskEXIT_CRITICAL();*/

	return _rssi;
}

/**
 * send a packet over the air.
 *
 * After sending the packet, the module goes to receive mode.
 * CSMA/CA is used before sending if enabled by function setCSMA() (default: off).
 *
 * @note A maximum amount of RFM69_MAX_PAYLOAD bytes can be sent.
 * @note This function blocks until packet has been sent.
 *
 * @param data Pointer to buffer with data
 * @param dataLength Size of buffer
 *
 * @return Number of bytes that have been sent
 */

int rfm69_send(const void* data, uint32_t dataLength)
{

	if(xSemaphoreTake(Radio_mutex, portMAX_DELAY)){ // dont interupt the sending

	if(RFM69_MODE_STANDBY != _rfm69_mode) //go to standby if necesary
	{
		rfm69_setMode(RFM69_MODE_STANDBY);
		rfm69_waitForModeReady();
	}

	//rfm69_writeRegister(0x28, 0x10); //clear FIFO

	// Restart RX
	rfm69_writeRegister(0x3D, (rfm69_readRegister(0x3D) & 0xFB) | 0x20);

	// switch to RX mode
	rfm69_setMode(RFM69_MODE_RX);

	vTaskDelay(1); //measure RSSI

	//uint32_t ulNotifiedValue;

	while (rfm69_readRSSI() >= CSMA_LIMIT) //check  if the chanel is free
	{
		//trace_printf("A\n");
		/*taskENTER_CRITICAL();
		trace_printf("wfn\n");
		taskEXIT_CRITICAL();*/
		//give back the mutex so the rx thread can receive data
		xSemaphoreGive(Radio_mutex);

		uint32_t not_val = ulTaskNotifyTake( pdTRUE, 3); //wait for 3 ms

		if( not_val ) //if payload is ready
		{
			/*taskENTER_CRITICAL();
			trace_printf("breaked\n");
			taskEXIT_CRITICAL();*/
			//data was received so reclaim the mutex
			xSemaphoreTake(Radio_mutex, portMAX_DELAY);

			//restart rx
			rfm69_writeRegister(0x3D, (rfm69_readRegister(0x3D) & 0xFB) | 0x20);

			break;
		}

		//no data was received so reclaim the mutex
		xSemaphoreTake(Radio_mutex, portMAX_DELAY);

		//and measure rsii
		//rfm69_writeRegister(0x3D, (rfm69_readRegister(0x3D) & 0xFB) | 0x20);

		//vTaskDelay(1); //measure RSSI

		//trace_printf("B\n");
	}

	//chanel is free

	//switch to standby and wait for mode ready, if not in sleep mode

  if (RFM69_MODE_SLEEP != _rfm69_mode)
  {
	  if(_rfm69_mode != RFM69_MODE_STANDBY)
	  {
		rfm69_setMode(RFM69_MODE_STANDBY);
		rfm69_waitForModeReady();
	  }
  }

  /*taskENTER_CRITICAL();
  			trace_printf("mua\n");
  			taskEXIT_CRITICAL();*/

  // payload must be available
  if (0 == dataLength)
    return 0;

  uint32_t len;
  uint32_t buflen;
  if (dataLength > RFM69_MAX_FIFO)
	  buflen = RFM69_MAX_FIFO;
  else
	  buflen = dataLength;

  // transfer packet to FIFO
  rfm69_chipSelect();

  // address FIFO
  uint8_t fifo = (REG_FIFO | 0x80);
  HAL_SPI_Transmit(&_rfm69_spi, &fifo , 1, HAL_MAX_DELAY);
  fifo = ((uint8_t)buflen)+1;
  HAL_SPI_Transmit(&_rfm69_spi, &fifo , 1, HAL_MAX_DELAY);

  // send payload
  for (len = 0; len <= buflen; len++)
  {
	  uint8_t dataBuffer = ((uint8_t*)data)[len];
	  HAL_SPI_Transmit(&_rfm69_spi, &dataBuffer , 1, HAL_MAX_DELAY);
  }

  rfm69_chipUnselect();

  // start radio transmission
  rfm69_setMode(RFM69_MODE_TX);

  /*while (len < dataLength)
  {
	  while ((rfm69_readRegister(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFONOTEMPTY) == 0x00)
	  {
		  if (len <= dataLength)
		  {
			  rfm69_chipSelect();
			  HAL_SPI_Transmit(&_rfm69_spi, &fifo , 1, HAL_MAX_DELAY);
			  uint8_t dataBuffer = ((uint8_t*)data)[len];
			  HAL_SPI_Transmit(&_rfm69_spi, &dataBuffer , 1, HAL_MAX_DELAY);
			  rfm69_chipUnselect();
			  len++;
		  }
	  }
  }*/
  // wait for packet sent
  rfm69_waitForPacketSent();


  rfm69_setMode(RFM69_MODE_STANDBY);
  rfm69_waitForModeReady();

  // go to rx
  rfm69_setMode(RFM69_MODE_RX);
	xSemaphoreGive(Radio_mutex);
	}

  return dataLength;
}

/**
 * Switch the mode of the RFM69 module.
 * Using this function you can manually select the RFM69 mode (sleep for example).
 *
 * This function also takes care of the special registers that need to be set when
 * the RFM69 module is a high power device (RFM69Hxx).
 *
 * This function is usually not needed because the library handles mode changes automatically.
 *
 * @param mode RFM69_MODE_SLEEP, RFM69_MODE_STANDBY, RFM69_MODE_FS, RFM69_MODE_TX, RFM69_MODE_RX
 * @return The new mode
 */
RFM69Mode rfm69_setMode(RFM69Mode mode)
{
  if ((mode == _rfm69_mode) || (mode > RFM69_MODE_RX))
    return _rfm69_mode;

  // set new mode
  rfm69_writeRegister(0x01, mode << 2);

  // set special registers if this is a high power device (RFM69HW)
  if (_rfm69_highPowerDevice > 0)
  {
    switch (mode)
    {
    case RFM69_MODE_RX:
      // normal RX mode
      if (_rfm69_highPowerSettings > 0)
        rfm69_setHighPowerSettings(0);
      break;

    case RFM69_MODE_TX:
      // +20dBm operation on PA_BOOST
      if (_rfm69_highPowerSettings > 0)
        rfm69_setHighPowerSettings(1);
      break;

    default:
      break;
    }
  }

  _rfm69_mode = mode;

  return _rfm69_mode;
}

/**
 * Wait until the requested mode is available or timeout.
 */
void rfm69_waitForModeReady()
{

	if(HAL_GPIO_ReadPin(RADIO_MODEREADY_GPIO_Port, RADIO_MODEREADY_Pin)==GPIO_PIN_RESET)
	{
		if(xSemaphoreTake(ModeReady_sem, 5)) //wait for 5 ms for the next interupt
		{
			return;
		}
		else
		{
			//if the isr had failed just pool every 1ms
			while(HAL_GPIO_ReadPin(RADIO_MODEREADY_GPIO_Port, RADIO_MODEREADY_Pin)==GPIO_PIN_RESET)
			{
				vTaskDelay(1);
			}
			return;
		}
	}
}

/**
 * Wait until packet has been sent over the air or timeout.
 */
void rfm69_waitForPacketSent()
{
	rfm69_writeRegister(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // set DIO0 to "Packet Sent" in transmit mode

	if(HAL_GPIO_ReadPin(RADIO_INT_GPIO_Port, RADIO_INT_Pin)==GPIO_PIN_RESET) //if packet was not already sent
	{
		if(xSemaphoreTake(PacketSent_sem, 5)) //wait for 5 ms for the next interupt
		{
			rfm69_writeRegister(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
			return;
		}
		else
		{
			//if the isr had failed for some reason just pool every 1ms
			while(HAL_GPIO_ReadPin(RADIO_INT_GPIO_Port, RADIO_INT_Pin)==GPIO_PIN_RESET)
			{
				vTaskDelay(1);
			}
			rfm69_writeRegister(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
			return;
		}
	}

}

/**
 * Put the RFM69 module to sleep (lowest power consumption).
 */
void rfm69_sleep()
{
  rfm69_setMode(RFM69_MODE_SLEEP);
}

/**
 * Set the carrier frequency in Hz.
 * After calling this function, the module is in standby mode.
 *
 * @param frequency Carrier frequency in Hz
 */
void rfm69_setFrequency(uint32_t frequency)
{
  // switch to standby if TX/RX was active
  RFM69Mode oldMode = _rfm69_mode;
  if (oldMode == RFM69_MODE_TX) {
    rfm69_setMode(RFM69_MODE_RX);
  }

  // calculate register value
  frequency /= RFM69_FSTEP;

  // set new frequency
  rfm69_writeRegister(0x07, frequency >> 16);
  rfm69_writeRegister(0x08, frequency >> 8);
  rfm69_writeRegister(0x09, frequency);

  if (oldMode == RFM69_MODE_RX) {
	  rfm69_setMode(RFM69_MODE_FS);
  }
  rfm69_setMode(oldMode);
}

// return the frequency (in Hz)
uint32_t rfm69_getFrequency()
{
  uint32_t freq;
  freq = RFM69_FSTEP * (((uint32_t) rfm69_readRegister(0x07) << 16) + ((uint16_t) rfm69_readRegister(0x08) << 8) + rfm69_readRegister(0x09));
  return freq;
}

/**
 * Set the FSK frequency deviation in Hz.
 * After calling this function, the module is in standby mode.
 *
 * @param frequency Frequency deviation in Hz
 */
void rfm69_setFrequencyDeviation(unsigned int frequency)
{
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == _rfm69_mode || RFM69_MODE_TX == _rfm69_mode)
    rfm69_setMode(RFM69_MODE_STANDBY);

  // calculate register value
  frequency /= RFM69_FSTEP;

  // set new frequency
  rfm69_writeRegister(0x05, frequency >> 8);
  rfm69_writeRegister(0x06, frequency);
}

/**
 * Set the bitrate in bits per second.
 * After calling this function, the module is in standby mode.
 *
 * @param bitrate Bitrate in bits per second
 */
void rfm69_setBitrate(unsigned int bitrate)
{
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == _rfm69_mode || RFM69_MODE_TX == _rfm69_mode)
    rfm69_setMode(RFM69_MODE_STANDBY);

  // calculate register value
  bitrate = RFM69_XO / bitrate;

  // set new bitrate
  rfm69_writeRegister(0x03, bitrate >> 8);
  rfm69_writeRegister(0x04, bitrate);
}

/**
 * Put the RFM69 module in RX mode and try to receive a packet.
 *
 * @note This is an internal function.
 * @note The module resides in RX mode.
 *
 * @param data Pointer to a receiving buffer
 * @param dataLength Maximum size of buffer
 * @return Number of received bytes; 0 if no payload is available.
 */
uint8_t rfm69_receive(char* data)
{

  // go to RX mode if not already in this mode
  if (RFM69_MODE_RX != _rfm69_mode)
  {
    rfm69_setMode(RFM69_MODE_RX);
    rfm69_waitForModeReady();
  }

  // check for flag PayloadReady
  if (rfm69_readRegister(0x28) & 0x04)
  {
	  unsigned int bytesRead = 0;

		// go to standby before reading data
		rfm69_setMode(RFM69_MODE_STANDBY);

		// get FIFO content


		uint8_t packet_size = rfm69_readRegister(0x00)-1;
		if(packet_size>100)
		{
			return 0;
		}

		for(bytesRead = 0; bytesRead<packet_size; bytesRead++)
		{
		  // read next byte
		  data[bytesRead] = rfm69_readRegister(0x00);
		}
		data[bytesRead] = 0;

		//rfm69_writeRegister(0x28, 0x10); //clear FIFO

    return bytesRead;
  }
  else
    return 0;
}
