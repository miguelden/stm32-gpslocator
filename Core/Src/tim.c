/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* TIM10 init function */
void MX_TIM10_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM10);

  TIM_InitStruct.Prescaler = 16799;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM10, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM10);

}

/* USER CODE BEGIN 1 */

/**
 * @brief  This starts the TIM10
 */
void MX_TIM10_Start(void)
{
    volatile uint32_t tmpcr1;
    tmpcr1 = READ_REG(TIM10->CR1);
    tmpcr1 |= TIM_CR1_CEN;
    WRITE_REG(TIM10->CR1, tmpcr1);
}

/**
 * @brief  This returns the current timer value
 * @return The current timer value
 */
uint32_t MX_TIM10_Get(void)
{
    return LL_TIM_GetCounter(TIM10);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
