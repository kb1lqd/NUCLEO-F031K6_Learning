/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include <stdbool.h>

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

volatile int rotary_val=0;

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin|LCD_EN_Pin|LCD_DB4_Pin|LCD_DB5_Pin
                          |LCD_DB6_Pin|LCD_DB7_Pin|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin
                           PAPin PAPin PA12 */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_EN_Pin|LCD_DB4_Pin|LCD_DB5_Pin
                          |LCD_DB6_Pin|LCD_DB7_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = PMOD_A_Pin|PMOD_BTN_Pin|PMOD_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 2 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	/*
	 *
	 * Sense Rotary Encoder Direction
	 *
	 */
	if(GPIO_Pin == PMOD_BTN_Pin)
	{
		__NOP(); // Push-Button pressed
	}
	else if(GPIO_Pin == PMOD_A_Pin)
	{
		__NOP(); // A signal detected

		//Sense direction
		if(b==true)
		{
			rotary_val++;
			a = false;
			b = false;
		}
		else
		{
			a = true;
		}



	}
	else if(GPIO_Pin == PMOD_B_Pin)
	{
		__NOP(); // B signal detected

		//Sense direction
		if(a==true)
		{
			rotary_val--;
			a = false;
			b = false;
		}
		else
		{
			b = true;
		}
	}

	__NOP();
	// End Sense Rotary Encoder Direction

}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
