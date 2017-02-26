/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */

typedef enum {
	SD_100KHZ = SPI_BAUDRATEPRESCALER_256,
	SD_20MHZ = SPI_BAUDRATEPRESCALER_2
} SD_SPEED;

/* Definitions for MMC/SDC command */
#define CMD0    (0x40+0)    /* GO_IDLE_STATE */
#define CMD1    (0x40+1)    /* SEND_OP_COND */
#define CMD8    (0x40+8)    /* SEND_IF_COND */
#define CMD9    (0x40+9)    /* SEND_CSD */
#define CMD10    (0x40+10)    /* SEND_CID */
#define CMD12    (0x40+12)    /* STOP_TRANSMISSION */
#define CMD16    (0x40+16)    /* SET_BLOCKLEN */
#define CMD17    (0x40+17)    /* READ_SINGLE_BLOCK */
#define CMD18    (0x40+18)    /* READ_MULTIPLE_BLOCK */
#define CMD23    (0x40+23)    /* SET_BLOCK_COUNT */
#define CMD24    (0x40+24)    /* WRITE_BLOCK */
#define CMD25    (0x40+25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD41    (0x40+41)    /* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)    /* APP_CMD */
#define CMD58    (0x40+58)    /* READ_OCR */

static BYTE CardType; /* b0:MMC, b1:SDC, b2:Block addressing */

extern SPI_HandleTypeDef hspi2;
void SPI_setSpeed(SD_SPEED speed)
{
	hspi2.Init.BaudRatePrescaler = speed;
	HAL_SPI_Init(&hspi2);
}

uint8_t SPI_send_single(uint8_t data)
{
	unsigned char TX, RX;
	TX = data;
	RX = 0;
	while ((HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY))
		;
	HAL_SPI_TransmitReceive(&hspi2, &TX, &RX, 1, 5000);

	return RX;
}

uint8_t SPI_receive_single() {
	unsigned char TX, RX;
	TX = 0xFF;
	RX = 0;
	while ((HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY))
		;
	HAL_SPI_TransmitReceive(&hspi2, &TX, &RX, 1, 5000);

	return RX;
}

void SPI_send(uint8_t* data, uint32_t length) {
	/*if (sdDMATCSemaphore) { // if semaphore was created, then use dma transfers
		stm32_dma_transfer(false, data, length);
	}
	else {*/
		while (length--) {
			SPI_send_single(*data);
			data++;
		}
	//}
}

void SPI_receive(uint8_t* data, uint32_t length) {
	/*if (sdDMATCSemaphore) { // if semaphore was created, then use dma transfers
		stm32_dma_transfer(true, data, length);
	}
	else {*/
		while (length--) {
			*data = SPI_receive_single();
			data++;
		}
	//}
}

BYTE wait_ready(void) {
	BYTE res;

	SPI_receive_single();
	do
		res = SPI_receive_single();
	while (res != 0xFF);

	return res;
}

static
void rcvr_spi_m(BYTE *dst) {
	*dst = SPI_receive_single();
}

#define TRUE  1
#define FALSE 0
#define bool BYTE
static bool rcvr_datablock(BYTE *buff, /* Data buffer to store received data */
UINT btr /* Byte count (must be even number) */
) {
	BYTE token;

	do { /* Wait for data packet in timeout of 100ms */
		token = SPI_receive_single();
	} while ((token == 0xFF));
	if (token != 0xFE)
		return FALSE; /* If not valid data token, return with error */

	do { /* Receive the data block into buffer */
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
	} while (btr -= 2);
	SPI_receive_single(); /* Discard CRC */
	SPI_receive_single();

	return TRUE; /* Return with success */
}

/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
static bool xmit_datablock(const BYTE *buff, /* 512 byte data block to be transmitted */
BYTE token /* Data/Stop token */
) {
	BYTE resp, wc;
	uint32_t i = 0;

	if (wait_ready() != 0xFF)
		return FALSE;

	SPI_send_single(token); /* Xmit data token */
	if (token != 0xFD) { /* Is data token */
		wc = 0;
		do { /* Xmit the 512 byte data block to MMC */
			SPI_send_single(*buff++);
			SPI_send_single(*buff++);
		} while (--wc);

		SPI_receive_single();
		SPI_receive_single();

		while (i <= 64) {
			resp = SPI_receive_single(); /* Reveive data response */
			if ((resp & 0x1F) == 0x05) /* If not accepted, return with error */
				break;
			i++;
		}
		while (SPI_receive_single() == 0)
			;
	}
	if ((resp & 0x1F) == 0x05)
		return TRUE;
	else
		return FALSE;
}
#endif /* _READONLY */

unsigned char sdcCommand[6];
void sdc_sendCommand(uint8_t command, uint8_t par1, uint8_t par2, uint8_t par3, uint8_t par4) {
	sdcCommand[0] = 0x40 | command;
	sdcCommand[1] = par1;
	sdcCommand[2] = par2;
	sdcCommand[3] = par3;
	sdcCommand[4] = par4;

	if (command == SDC_GO_IDLE_STATE) { // TODO: crc calculation and validation
		sdcCommand[5] = 0x95; // precalculated CRC
	}
	else {
		sdcCommand[5] = 0xFF;
	}
	SPI_send(sdcCommand, 6);
}

uint8_t sdc_getResponse(uint8_t response) {
	for(uint8_t n = 0; n < 8; n++) {
		if (SPI_receive_single() == response) {
			return 0;
		}
	}
	return 1;
}

BYTE send_cmd(BYTE cmd, /* Command byte */
DWORD arg /* Argument */
) {
	BYTE n, res;

	if (wait_ready() != 0xFF)
		return 0xFF;

	/* Send command packet */
	SPI_send_single(cmd); /* Command */
	SPI_send_single((BYTE) (arg >> 24)); /* Argument[31..24] */
	SPI_send_single((BYTE) (arg >> 16)); /* Argument[23..16] */
	SPI_send_single((BYTE) (arg >> 8)); /* Argument[15..8] */
	SPI_send_single((BYTE) arg); /* Argument[7..0] */
	n = 0;
	if (cmd == CMD0)
		n = 0x95; /* CRC for CMD0(0) */
	if (cmd == CMD8)
		n = 0x87; /* CRC for CMD8(0x1AA) */
	SPI_send_single(n);

	/* Receive command response */
	if (cmd == CMD12)
		SPI_receive_single(); /* Skip a stuff byte when stop reading */
	n = 10; /* Wait for a valid response in timeout of 10 attempts */
	do
		res = SPI_receive_single();
	while ((res & 0x80) && --n);

	return res; /* Return with the response value */
}

void sdc_assert(void) {
	HAL_GPIO_WritePin(SD_SS_GPIO_Port, SD_SS_Pin, GPIO_PIN_RESET);
}

void sdc_deassert(void) {
	HAL_GPIO_WritePin(SD_SS_GPIO_Port, SD_SS_Pin, GPIO_PIN_SET);
	SPI_send_single(0xFF); // send 8 clocks for SDC to set SDO tristate
}

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive number (0..) */
)
{
	HAL_GPIO_WritePin(SD_SCK_GPIO_Port, SD_SCK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SD_MOSI_GPIO_Port, SD_MOSI_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SD_MISO_GPIO_Port, SD_MISO_Pin, GPIO_PIN_RESET);
	sdc_assert();

	// delay aprox. 1ms
	volatile uint32_t counter;
	for(counter = 0; counter<50000; counter++);

	// de-assert CS line
	HAL_GPIO_WritePin(SD_SS_GPIO_Port, SD_SS_Pin, GPIO_PIN_SET);

	// init SPI
	SPI_setSpeed(SD_100KHZ);

	// send 10 dummy bytes to wake up SDC
	for (uint8_t i = 0; i < 10; i++) {
		SPI_send_single(0xFF);
	}

	for(counter = 0; counter<50000; counter++);

	// assert SDC
	sdc_assert();

	unsigned char i, cmd_arg[6];
	unsigned int Count = 0x1FFF;

	cmd_arg[0] = (CMD0 | 0x40);
	cmd_arg[1] = 0;
	cmd_arg[2] = 0;
	cmd_arg[3] = 0;
	cmd_arg[4] = 0;
	cmd_arg[5] = 0x95;

	for (i = 0; i < 6; i++)
		SPI_send_single(cmd_arg[i]);

	while ((SPI_receive_single() != 0x01) && Count)
		Count--;

	sdc_deassert();

	BYTE n, ty, ocr[4];

	sdc_assert(); /* CS = L */
	ty = 0;
	if (send_cmd(CMD0, 0) == 1) { /* Enter Idle state */
		if (send_cmd(CMD8, 0x1AA) == 1) { /* SDC Ver2+ */
			for (n = 0; n < 4; n++)
				ocr[n] = SPI_receive_single();
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) { /* The card can work at vdd range of 2.7-3.6V */
				do {
					if (send_cmd(CMD55, 0) <= 1
							&& send_cmd(CMD41, 1UL << 30) == 0)
						break; /* ACMD41 with HCS bit */
				} while (1);
				if (send_cmd(CMD58, 0) == 0) { /* Check CCS bit */
					for (n = 0; n < 4; n++)
						ocr[n] = SPI_receive_single();
					ty = (ocr[0] & 0x40) ? 6 : 2;
				}
			}
		} else { /* SDC Ver1 or MMC */
			ty = (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 0) <= 1) ? 2 : 1; /* SDC : MMC */
			do {
				if (ty == 2) {
					if (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 0) == 0)
						break; /* ACMD41 */
				} else {
					if (send_cmd(CMD1, 0) == 0)
						break; /* CMD1 */
				}
			} while (1);
			//if (!Timer1 || send_cmd(CMD16, 512) != 0) /* Select R/W block length */
			//	ty = 0;
		}
	}
	sdc_deassert();

	for(counter = 0; counter<50000; counter++);

	CardType = ty;
	SPI_setSpeed(SD_20MHZ);

	if (ty)
		/* Initialization succeded */
		return 0;
	else
		/* Initialization failed */
		return STA_NOINIT;

}


/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{
	/*
	sdc_assert();
	sdc_sendCommand(SDC_SEND_STATUS, 0, 0, 0, 0);
	SPI_receive(SPI1, result, 2);
	sdc_deassert();

	if (result[0] & 0x01)
	{
		return STA_NOINIT;
	}
	else if (result[1] & 0x01)
	{
		return STA_PROTECT;
	}
	*/

	return 0;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	if (!(CardType & 4))
		sector *= 512; /* Convert to byte address if needed */

	sdc_assert(); /* CS = L */

	if (count == 1) { /* Single block read */
		if ((send_cmd(CMD17, sector) == 0) /* READ_SINGLE_BLOCK */
		&& rcvr_datablock(buff, 512))
			count = 0;
	} else { /* Multiple block read */
		if (send_cmd(CMD18, sector) == 0) { /* READ_MULTIPLE_BLOCK */
			do {
				if (!rcvr_datablock(buff, 512))
					break;
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0); /* STOP_TRANSMISSION */
		}
	}

	sdc_deassert();

	return count ? RES_ERROR : RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE* buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	if (!(CardType & 4))
			sector *= 512; /* Convert to byte address if needed */

		sdc_assert(); /* CS = L */

		if (count == 1) { /* Single block write */
			if ((send_cmd(CMD24, sector) == 0) /* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE))
				count = 0;
		} else { /* Multiple block write */
			if (CardType & 2) {
				send_cmd(CMD55, 0);
				send_cmd(CMD23, count); /* ACMD23 */
			}
			if (send_cmd(CMD25, sector) == 0) { /* WRITE_MULTIPLE_BLOCK */
				do {
					if (!xmit_datablock(buff, 0xFC))
						break;
					buff += 512;
				} while (--count);
				if (!xmit_datablock(0, 0xFD)) /* STOP_TRAN token */
					count = 1;
			}
		}

		sdc_deassert();

		return count ? RES_ERROR : RES_OK;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	return RES_OK;
}
#endif

DWORD get_fattime (void)
{
	return 0;
}
