/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef struct
{
	float Kp;
	float Ki;
	float Kd; 
	float I_part;
	float Error;
	float pre_Error;
	float Output;
} PID;

typedef struct
{
		uint16_t pulse;
		uint16_t vel;
		uint8_t dir;
		char cd;
		uint16_t iw;
}	bot;

typedef struct
{
	float w;
	uint32_t pos;
	uint32_t pre_pos;
} encoder;
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void timeinterrupt(void);
void vel_convert(char* vel_pub);
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STBY_Pin GPIO_PIN_13
#define STBY_GPIO_Port GPIOC
#define IN31_Pin GPIO_PIN_14
#define IN31_GPIO_Port GPIOC
#define IN32_Pin GPIO_PIN_15
#define IN32_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_15
#define PWM1_GPIO_Port GPIOA
#define IN12_Pin GPIO_PIN_1
#define IN12_GPIO_Port GPIOA
#define IN11_Pin GPIO_PIN_5
#define IN11_GPIO_Port GPIOA
#define EN3A_Pin GPIO_PIN_6
#define EN3A_GPIO_Port GPIOA
#define EN3B_Pin GPIO_PIN_7
#define EN3B_GPIO_Port GPIOA
#define IN21_Pin GPIO_PIN_0
#define IN21_GPIO_Port GPIOB
#define IN22_Pin GPIO_PIN_12
#define IN22_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_10
#define PWM2_GPIO_Port GPIOB
#define PWM3_Pin GPIO_PIN_11
#define PWM3_GPIO_Port GPIOB
#define SS_Pin GPIO_PIN_12
#define SS_GPIO_Port GPIOB
#define SCK_Pin GPIO_PIN_13
#define SCK_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_14
#define MISO_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_15
#define MOSI_GPIO_Port GPIOB
#define EN1A_Pin GPIO_PIN_8
#define EN1A_GPIO_Port GPIOA
#define EN1B_Pin GPIO_PIN_9
#define EN1B_GPIO_Port GPIOA
#define EN2A_Pin GPIO_PIN_6
#define EN2A_GPIO_Port GPIOB
#define EN2B_Pin GPIO_PIN_7
#define EN2B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
