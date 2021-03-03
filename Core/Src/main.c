/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include "app.h"
#include "userif.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DMA_RX_BUFFER_SIZE   1024UL

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/** DMA UART RX Buffer */
uint8_t dma_rx_buffer_[DMA_RX_BUFFER_SIZE];
/** RX Buffer */
uint8_t rx_buffer_[DMA_RX_BUFFER_SIZE];
/** Rx Buffer Pointer. Last index to be read */
uint32_t rx_buffer_end_ = 0;
/** Rx Buffer Pointer. Next index to read. If it is equals to 'rx_buffer_end' -> nothing to read */
uint32_t rx_buffer_0_ = 0;

/** Target on range flag */
uint8_t target_on_range_ = 0;
/** GPS fix flag */
uint8_t gps_fix_ = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
 * @brief  UART-DMA RX Transfer completed callback
 */
void uart_read_dma_callback (

)
{
    static size_t idma_0;
    size_t idma_f;

    /* Get idx in DMA circular buffer */
    idma_f = DMA_RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5);
    if (idma_f != idma_0) {

    	size_t sz = (DMA_RX_BUFFER_SIZE + idma_f - idma_0) % DMA_RX_BUFFER_SIZE;
    	size_t capacity = (DMA_RX_BUFFER_SIZE - 1 + rx_buffer_0_ - rx_buffer_end_) % DMA_RX_BUFFER_SIZE;
    	size_t last_idx = (idma_f == DMA_RX_BUFFER_SIZE) ? 0 : idma_f;

    	if(sz < capacity) {

            /* Linear reading */
            if (idma_f > idma_0) {

            	memcpy(&rx_buffer_[idma_0], &dma_rx_buffer_[idma_0], idma_f - idma_0);

                /* Overflowed reading */
            } else {
                memcpy(&rx_buffer_[idma_0], &dma_rx_buffer_[idma_0], DMA_RX_BUFFER_SIZE - idma_0);
                if (idma_f > 0) {
                    memcpy(&rx_buffer_[0], &dma_rx_buffer_[0], idma_f);
                }
            }

            /* Update last index to be read */
            rx_buffer_end_ = last_idx;
    	}

        /* Save current index */
        idma_0 = last_idx;
    }
}

/**
 * @brief   UART-DMA TX Transfer completed callback
 */
void uart_dma_tx_complete_callback (

)
{

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint16_t t0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  /* Start timer */
  MX_TIM10_Start();

  /* Set DMA RX buffer */
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)dma_rx_buffer_);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, DMA_RX_BUFFER_SIZE);
  MX_USART2_UART_Enable();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  t0 = MX_TIM10_Get();
  while (1)
  {
    size_t rx_end = rx_buffer_end_;
    uint16_t t1;
    uint32_t dt;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*
     * Read new character
     */
    if (rx_buffer_0_!=rx_end) {
      app_step(rx_buffer_[rx_buffer_0_]);
      rx_buffer_0_ = (rx_buffer_0_ + 1) % DMA_RX_BUFFER_SIZE;
    }


    /*
     * LED Management
     * Period = 250ms -> 25000 cnt in TMR10
     */

    /* Get current time */
    t1 = (uint16_t)MX_TIM10_Get();
    dt = (0x10000UL - t0 + t1 + 1) & 0xFFFF;
    if (dt > 2500) {

      /* System LED (Red) */
      LL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

      /* GPS FIX LED (Blue) */
      if (userif_get_gps_status()) {
        LL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
      } else {
      	LL_GPIO_ResetOutputPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
      }

      /* TARGET On Range LED (Green) */
      if (userif_get_target_reached()) {
        LL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
      } else {
      	LL_GPIO_ResetOutputPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
      }

      t0 = t1;

    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(168000000);
  LL_SetSystemCoreClock(168000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
