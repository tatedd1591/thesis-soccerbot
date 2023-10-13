/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pwm.c
  * @brief          : Control motor
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
	*note: motor
	STBY: PC13
	*motor 1:
	PA0 - PA15: xung
	PA1: IN1 -> HIGH: tien
	PA5: IN2 -> HIGH: lùi
	encoder: tim1
	*motor 2:
	PB10: xung
	PB0: IN21 -> HIGH: tien
	PB1: IN22 -> HIGH: lùi
	encoder: tim4
	*motor 3:
	PB11: xung
	PC14: IN31 -> HIGH: tien
	PC15: IN32 -> HIGH: lùi
	encoder: tim3
	xung: tim2
	*motor 0:
	PB3: xung
	PC14: IN31 -> HIGH: tien
	PC15: IN32 -> HIGH: lùi
	encoder: tim3
	xung: tim2
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/	
#include "pwm.h"
#include "stm32f1xx_hal.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/**
  * @brief Reading encoder motor
  * @retval void
  */
void encoder_reading(encoder* motor1,encoder* motor2,encoder* motor3)
{
	motor1->pre_pos=motor1->pos;
	motor2->pre_pos=motor2->pos;
	motor3->pre_pos=motor3->pos;
  motor1->pos=__HAL_TIM_GetCounter(&htim1);
	motor2->pos=__HAL_TIM_GetCounter(&htim4);
	motor3->pos=__HAL_TIM_GetCounter(&htim3);
}

/**
  * @brief Reading angular speed
  * @param previous position and current position of encoder
  * @unit RPM
  * @retval float
  */
float cal_vel(encoder* motor)
{	
	if (motor->pos > motor->pre_pos) 
	{
		if (motor->pos - motor->pre_pos > 30000) motor->w = (65535 - motor->pos + motor->pre_pos)*60/(1570*tpid);
		else motor->w =(motor->pos - motor->pre_pos)*60/(1570*tpid);
	}
	else if (motor->pre_pos == motor->pos) motor->w = 0;
	else 
	{
		if (motor->pre_pos - motor->pos > 30000) motor->w =(65535 - motor->pre_pos + motor->pos)*60/(1570*tpid);
		else motor->w =(motor->pre_pos - motor->pos)*60/(1570*tpid);
	}
	if (motor->w > 340) motor->w = 0;
  return motor->w;
}


/**
  * @brief Calculate PID speed value to control motor
  * @unit Speed percentage
  * @retval int16_t 
  */

int16_t PID_motor(PID* pid, float REF, encoder* motor)
{
	pid->pre_Error 	= pid->Error;
	pid->Error 			= REF-cal_vel(motor);
	pid->I_part 		= pid->I_part + pid->Error*tpid;
	pid->Output 		= pid->Kp*pid->Error + pid->Ki*pid->I_part + pid->Kd*(pid->Error - pid->pre_Error)/tpid;
	if (pid->Output>pulse_max) pid->Output=90;
	else if (pid->Output<pulse_min) pid->Output=0;
	return pid->Output;
}

/**
  * @brief  Set speed for motor.
  * @param  Speed percentage
  * @retval None
  */
void ballkeeping(uint8_t a)
{
	if (a==1)
	{
	htim2.Instance->CCR2 = 200;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
	}
	else if (a==2)
	{
	htim2.Instance->CCR2 = 200;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
	}
	else if (a==3)
	{
	htim2.Instance->CCR2 = 0;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
	}
}
void Speed1_Backward(uint16_t speed)
{	
	htim2.Instance->CCR1 = speed;
	HAL_GPIO_WritePin(IN12_GPIO_Port,IN12_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN11_GPIO_Port,IN11_Pin,GPIO_PIN_RESET);
}

void Speed1_Forward(uint16_t speed)
{
	htim2.Instance->CCR1 = speed;
	HAL_GPIO_WritePin(IN12_GPIO_Port,IN12_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN11_GPIO_Port,IN11_Pin,GPIO_PIN_SET);
}

void Speed1_Stop(void)
{
	htim2.Instance->CCR1 = 0;
	HAL_GPIO_WritePin(IN12_GPIO_Port,IN12_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN11_GPIO_Port,IN11_Pin,GPIO_PIN_RESET);
}

/**
  * @brief  Set speed for motor2.
  * @param  Speed percentage
  * @retval None
  */

void Speed2_Backward(uint16_t speed)
{
	htim2.Instance->CCR3 = speed;
	HAL_GPIO_WritePin(IN21_GPIO_Port,IN21_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN22_GPIO_Port,IN22_Pin,GPIO_PIN_RESET);
}
void Speed2_Forward(uint16_t speed)
{
	htim2.Instance->CCR3 = speed;
	HAL_GPIO_WritePin(IN21_GPIO_Port,IN21_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN22_GPIO_Port,IN22_Pin,GPIO_PIN_SET);
}

void Speed2_Stop(void)
{
	htim2.Instance->CCR3 = 0;
	HAL_GPIO_WritePin(IN21_GPIO_Port,IN21_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN22_GPIO_Port,IN22_Pin,GPIO_PIN_RESET);
}
/**
  * @brief  Set speed for motor2.
  * @param  Speed percentage
  * @retval None
  */

void Speed3_Forward(uint16_t speed)
{
	htim2.Instance->CCR4 = speed;
	HAL_GPIO_WritePin(IN31_GPIO_Port,IN31_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN32_GPIO_Port,IN32_Pin,GPIO_PIN_RESET);
}
void Speed3_Backward(uint16_t speed)
{
	htim2.Instance->CCR4 = speed;
	HAL_GPIO_WritePin(IN31_GPIO_Port,IN31_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN32_GPIO_Port,IN32_Pin,GPIO_PIN_SET);
}

void Speed3_Stop(void)
{
	htim2.Instance->CCR4 = 0;
	HAL_GPIO_WritePin(IN31_GPIO_Port,IN31_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN32_GPIO_Port,IN32_Pin,GPIO_PIN_RESET);
}
/**
  * @brief  Set speed to control motor.
  * @param  Speed percentage
  * @retval None
  */
void Stop(void) //dung
{
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_RESET);
}
void Forward_l(uint16_t speed1,uint16_t speed2,uint16_t speed3) //tien
{
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
	Speed1_Forward(speed1);
	Speed2_Forward(speed2);
	Speed3_Backward(speed3);
}
void Backward_l(uint16_t speed1,uint16_t speed2,uint16_t speed3) //tien
{
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
	Speed1_Backward(speed1);
	Speed2_Backward(speed2);
	Speed3_Forward(speed3);
}
void Forward_n(uint16_t speed1,uint16_t speed2,uint16_t speed3) //tien
{
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
	Speed1_Backward(speed1);
	Speed2_Forward(speed2);
	Speed3_Forward(speed3);
}
void Backward_n(uint16_t speed1,uint16_t speed2,uint16_t speed3) //tien
{
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
	Speed1_Forward(speed1);
	Speed2_Backward(speed2);
	Speed3_Backward(speed3);
}
void RotateCC(uint16_t speed1,uint16_t speed2, uint16_t speed3) //xoay tai cho nguoc chieu
{
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
	Speed2_Backward(speed1);
	Speed1_Backward(speed2);
	Speed3_Backward(speed3);
}
void Rotate(uint16_t speed1,uint16_t speed2, uint16_t speed3) //xoay tai cho cung chieu
{
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
	Speed1_Forward(speed1);
	Speed2_Forward(speed2);
	Speed3_Forward(speed3);
}
void control(uint16_t speed1,uint16_t speed2, uint16_t speed3,uint8_t a1,uint8_t a2, uint8_t a3)
{
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
	if (a1==0) Speed1_Forward(speed1);
	else Speed1_Backward(speed1);
	
	if (a2==0) Speed2_Forward(speed2);
	else Speed2_Backward(speed2);
	
	if (a3==0) Speed3_Forward(speed3);
	else Speed3_Backward(speed3);
}


