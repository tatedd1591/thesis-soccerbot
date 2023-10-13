/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pwm.h
  * @brief          : Header for pwm.c file.
  *                   This file contains the common defines of the application.
	* @author         : Duong Le Tuan Anh
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/
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
		int iw;
}	bot;

typedef struct
{
	float w;
	uint32_t pos;
	uint32_t pre_pos;
} encoder;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define tpid 		0.01
#define pulse_max 		90
#define pulse_min			0
/* Exported functions ------------------------------------------------------- */
void encoder_reading(encoder* motor1,encoder* motor2,encoder* motor3);
float cal_vel(encoder* motor);
int16_t PID_motor(PID* pid, float REF, encoder* motor);
void Speed1_Forward(uint16_t speed);
void Speed2_Forward(uint16_t speed);
void Speed3_Forward(uint16_t speed);
void Speed1_Backward(uint16_t speed);
void Speed2_Backward(uint16_t speed);
void Speed3_Backward(uint16_t speed);
void Speed1_Stop(void);
void Speed2_Stop(void);
void Speed3_Stop(void);
void Forward_l(uint16_t speed1,uint16_t speed2, uint16_t speed3);
void Forward_n(uint16_t speed1,uint16_t speed2, uint16_t speed3);
void Backward_l(uint16_t speed1,uint16_t speed2, uint16_t speed3);
void Backward_n(uint16_t speed1,uint16_t speed2, uint16_t speed3);
void Rotate(uint16_t speed1,uint16_t speed2, uint16_t speed3);
void RotateCC(uint16_t speed1,uint16_t speed2, uint16_t speed3);
void control(uint16_t speed1,uint16_t speed2, uint16_t speed3,uint8_t a1,uint8_t a2, uint8_t a3);
void Stop(void);
void TimerInit(void);
void TimerStop(void);
void ballkeeping(uint8_t a);


