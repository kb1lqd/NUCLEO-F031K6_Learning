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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
	static bool pmod_a=0;
	static bool pmod_b=0;
	bool pmod_turn_cw=0;
	bool pmod_turn_ccw=0;
	bool pmod_btn=0;

	if(GPIO_Pin == PMOD_BTN_Pin)
	{
		__NOP(); // Push-Button pressed
		pmod_btn=1;
	}
	else if(GPIO_Pin == PMOD_A_Pin)
	{
		__NOP(); // A signal detected
		pmod_a=1;

		//Determine direction
		if(pmod_b==1)
		{
			//CW
			pmod_turn_cw = 1;

			//RESET
			pmod_a=0;
			pmod_b=0;
			pmod_turn_cw = 0;
			pmod_turn_ccw = 0;
		}
	}
	else if(GPIO_Pin == PMOD_B_Pin)
	{
		__NOP(); // B signal detected
		pmod_b=1;

		//Determine direction
		if(pmod_a==1)
		{
			//CCW
			pmod_turn_ccw = 1;

			//RESET
			pmod_a=0;
			pmod_b=0;
			pmod_turn_cw = 0;
			pmod_turn_ccw = 0;
		}
	}

	//Determine direction

}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
