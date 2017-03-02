/*
 * cansat.c
 *
 *  Created on: 07.02.2017
 *      Author: grzegorz
 */

#include <cansat.h>

FATFS sdCardFS;

FIL radio_log;

TaskHandle_t tsk_radio_rx;
TaskHandle_t tsk_radio_tx;
TaskHandle_t tsk_barometer;

SemaphoreHandle_t PayloadReady_sem;
SemaphoreHandle_t PacketSent_sem;
SemaphoreHandle_t ModeReady_sem;
SemaphoreHandle_t Radio_mutex;

QueueHandle_t Radio_echo;
QueueHandle_t Barometer_telemetry;

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    signed char *pcTaskName )
{
	trace_printf("StackFail: %s\n", pcTaskName);
	for(;;);
}

void CreateSynchronizationObjects()
{
	//create synchronization objects
	vSemaphoreCreateBinary(PayloadReady_sem);
	vSemaphoreCreateBinary(PacketSent_sem);
	vSemaphoreCreateBinary(ModeReady_sem);
	Radio_mutex = xSemaphoreCreateMutex();

	Radio_echo = xQueueCreate(256, sizeof(char));
	Barometer_telemetry = xQueueCreate(32, sizeof(float));

}

void CanSatMain()
{
	//initialize the file system
	/*while (f_mount(&sdCardFS, "0:/", 0) != FR_OK)
	{
		trace_printf("Failed to init SD-Card");
	}

	while (f_open(&radio_log, "radio.log", FA_WRITE | FA_OPEN_APPEND) != FR_OK)
	{
		trace_printf("Failed to open radio log");
	}*/

	rfm69_init(hspi1, 1);
	rfm69_setPowerDBm(-2);
	rfm69_setFrequency(433500000);
	rfm69_setMode(RFM69_MODE_RX);
	//rfm69_sleep();

	trace_printf("Radio initialized\n");

	/*uint8_t i = 0;
	for(i = 0x00; i<255; i++)
	{
	    if(HAL_I2C_IsDeviceReady(&hi2c2, i, 2, 10) == HAL_OK)
			trace_printf("Ready: 0x%02x\n", i);
	    else
	    	trace_printf("WRONG: 0x%02x\n", i);
	}*/

	while(bmp180_initialize(BMP180_ULTRAHIGHRES, &hi2c1)==0)
	{
		trace_printf("Failed to initialize barometer\n");
	}

	trace_printf("Barometer initialized\n");

	/*while(1)
	{
		//uint32_t pres = bmp180_readPressure();
		float temp = bmp180_readTemperature();
		//float alt = bmp180_readAltitude(BMP180_STANDARD_PRESURE);
		trace_printf("%d.%d\n", (int)temp, (int)(temp*100)-(int)temp*100);
		HAL_Delay(100);
	}*/

	//start tasks
	xTaskCreate(HandleRadioRX, "radiorx", 128, NULL, tskIDLE_PRIORITY+1, &tsk_radio_rx);
	xTaskCreate(HandleRadioTX, "radiotx", 128, NULL, tskIDLE_PRIORITY+1, &tsk_radio_tx);
	xTaskCreate(HandleBarometer, "baro", 128, NULL, tskIDLE_PRIORITY+1, &tsk_barometer);

	//start FreeRTOS
	vTaskStartScheduler();


	/*
	//Benchmark of writing 1kB of data
	uint32_t time0 = HAL_GetTick();
	uint32_t wbytes;

	char buff[512*20];
	memset(buff, 'C', 512*20); //10 kb buffer

	if(f_open(&file, "bnch3", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
	{
		f_expand(&file, 100*1024, 1);
		for(int i = 0; i<30; i++)
		{
			/* Get physical location of the file data
			    BYTE drv = file.obj.fs->drv;
			    DWORD sect = file.obj.fs->database + file.obj.fs->csize * (file.obj.sclust - 2);

			    /* Write 2048 sectors from top of the file at a time
			    disk_write(drv, buff, sect, 20);
		}

		f_close(&file);

		uint32_t time1 = HAL_GetTick();

		//save test result
		if(f_open(&resfile, "res.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
		{
			f_printf(&resfile, "Writing 300kB of data took %d ms", time1-time0);
			f_close(&resfile);
		}

		trace_printf("Writing 300kB of data took %d ms", time1-time0);
	}
	else
	{
		trace_printf("U fckd up m8");
	}

	for(;;){}*/

	//HAL_GPIO_WritePin(MOTOR_DISABLE_GPIO_Port, MOTOR_DISABLE_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);

	/*for(;;)
	{

	}*/

	/*if (f_mount(&sdCard, "0:/", 0) != FR_OK) {
		trace_printf("Nie mozemy zamontowac karty SD :(\n");
		trace_printf("%s", "0:/");
	}
	if (f_open(&file, "ansAt.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
		trace_printf("Nie udalo sie utworzyc przykladowego pliku\n");
	}

	uint32_t messages = 0;
	uint32_t message_len = 0;
	uint32_t wbytes;
	while (1) {
		messages++;
		message_len = sprintf((char*)buffor, "ASDASDZapisalismy dotychczas %d wiadomosci\n", messages);
		if(f_write(&file, buffor, message_len, (void*)&wbytes) != FR_OK)
		{
			trace_printf("Nie udalo sie zapisac %d wiadomosci\n", messages);
		}
		f_sync(&file);
		trace_printf("%s", buf1);
		HAL_Delay(1000);
		if(messages==10)
			f_close(&file);

	}*/


	/*xTaskHandle lol;
	xTaskHandle lol2;

	xTaskCreate(lols, "Lol", 128, NULL, tskIDLE_PRIORITY+1, &lol);
	xTaskCreate(lols2, "Lol2", 128, NULL, tskIDLE_PRIORITY+1, &lol2);

	vTaskStartScheduler();*/
}


