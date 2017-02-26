/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MOTOR_DISABLE_Pin GPIO_PIN_0
#define MOTOR_DISABLE_GPIO_Port GPIOA
#define RADIO_INT_Pin GPIO_PIN_6
#define RADIO_INT_GPIO_Port GPIOB
#define RADIO_MODEREADY_Pin GPIO_PIN_7
#define RADIO_MODEREADY_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_2
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_3
#define GPS_RX_GPIO_Port GPIOA
#define RADIO_SS_Pin GPIO_PIN_4
#define RADIO_SS_GPIO_Port GPIOA
#define RADIO_SCK_Pin GPIO_PIN_5
#define RADIO_SCK_GPIO_Port GPIOA
#define RADIO_MISO_Pin GPIO_PIN_6
#define RADIO_MISO_GPIO_Port GPIOA
#define RADIO_MOSI_Pin GPIO_PIN_7
#define RADIO_MOSI_GPIO_Port GPIOA
#define RADIO_RESET_PIN GPIO_PIN_5
#define RADIO_RESET_GPIO_Port GPIOB
#define MOTOR_STEP_Pin GPIO_PIN_0
#define MOTOR_STEP_GPIO_Port GPIOB
#define MOTOR_DIR_Pin GPIO_PIN_1
#define MOTOR_DIR_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_10
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_11
#define IMU_SDA_GPIO_Port GPIOB
#define SD_SS_Pin GPIO_PIN_12
#define SD_SS_GPIO_Port GPIOB
#define SD_SCK_Pin GPIO_PIN_13
#define SD_SCK_GPIO_Port GPIOB
#define SD_MISO_Pin GPIO_PIN_14
#define SD_MISO_GPIO_Port GPIOB
#define SD_MOSI_Pin GPIO_PIN_15
#define SD_MOSI_GPIO_Port GPIOB
/*#define CAM_D0_Pin GPIO_PIN_8
#define CAM_D0_GPIO_Port GPIOA
#define CAM_D1_Pin GPIO_PIN_9
#define CAM_D1_GPIO_Port GPIOA
#define CAM_D2_Pin GPIO_PIN_10
#define CAM_D2_GPIO_Port GPIOA
#define CAM_D3_Pin GPIO_PIN_11
#define CAM_D3_GPIO_Port GPIOA
#define CAM_D4_Pin GPIO_PIN_12
#define CAM_D4_GPIO_Port GPIOA
#define CAM_D5_Pin GPIO_PIN_15
#define CAM_D5_GPIO_Port GPIOA
#define CAM_D6_Pin GPIO_PIN_3
#define CAM_D6_GPIO_Port GPIOB
#define CAM_D7_Pin GPIO_PIN_4
#define CAM_D7_GPIO_Port GPIOB
#define CAM_PCLK_Pin GPIO_PIN_5
#define CAM_PCLK_GPIO_Port GPIOB
#define CAM_VSYNC_Pin GPIO_PIN_6
#define CAM_VSYNC_GPIO_Port GPIOB
#define CAM_CLK_Pin GPIO_PIN_7
#define CAM_CLK_GPIO_Port GPIOB*/
#define LS_SCL_Pin GPIO_PIN_8
#define LS_SCL_GPIO_Port GPIOB
#define LS_SDA_Pin GPIO_PIN_9
#define LS_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
