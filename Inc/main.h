/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Sout6_Pin GPIO_PIN_13
#define Sout6_GPIO_Port GPIOC
#define Sout5_Pin GPIO_PIN_0
#define Sout5_GPIO_Port GPIOC
#define Sout4_Pin GPIO_PIN_1
#define Sout4_GPIO_Port GPIOC
#define Sout3_Pin GPIO_PIN_2
#define Sout3_GPIO_Port GPIOC
#define Sout2_Pin GPIO_PIN_3
#define Sout2_GPIO_Port GPIOC
#define Sout1_Pin GPIO_PIN_0
#define Sout1_GPIO_Port GPIOA
#define Sout0_Pin GPIO_PIN_1
#define Sout0_GPIO_Port GPIOA
#define RX_DISABLE__Pin GPIO_PIN_4
#define RX_DISABLE__GPIO_Port GPIOC
#define CLEAR__Pin GPIO_PIN_5
#define CLEAR__GPIO_Port GPIOC
#define IN1__Pin GPIO_PIN_0
#define IN1__GPIO_Port GPIOB
#define RASP_RESET_Pin GPIO_PIN_1
#define RASP_RESET_GPIO_Port GPIOB
#define TP1_Pin GPIO_PIN_12
#define TP1_GPIO_Port GPIOB
#define TP2_Pin GPIO_PIN_13
#define TP2_GPIO_Port GPIOB
#define LED1_VD7_Pin GPIO_PIN_14
#define LED1_VD7_GPIO_Port GPIOB
#define LED2_VD8_Pin GPIO_PIN_15
#define LED2_VD8_GPIO_Port GPIOB
#define ALARM_Pin GPIO_PIN_9
#define ALARM_GPIO_Port GPIOC
#define OUT1__Pin GPIO_PIN_8
#define OUT1__GPIO_Port GPIOA
#define HF_FAULT__Pin GPIO_PIN_9
#define HF_FAULT__GPIO_Port GPIOA
#define TEST_GOOSE__Pin GPIO_PIN_10
#define TEST_GOOSE__GPIO_Port GPIOA
#define WARNING__Pin GPIO_PIN_15
#define WARNING__GPIO_Port GPIOA
#define En_DR_Pin GPIO_PIN_12
#define En_DR_GPIO_Port GPIOC
#define COM_TR_Pin GPIO_PIN_2
#define COM_TR_GPIO_Port GPIOD
#define TM_RX_Pin GPIO_PIN_4
#define TM_RX_GPIO_Port GPIOB
#define TM_TX_Pin GPIO_PIN_5
#define TM_TX_GPIO_Port GPIOB
#define COM_RC_Pin GPIO_PIN_8
#define COM_RC_GPIO_Port GPIOB
#define Sout7_Pin GPIO_PIN_9
#define Sout7_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
