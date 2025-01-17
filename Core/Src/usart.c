/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "string.h"


#define DMA_RX_BUFFER_SIZE   1024UL

/** DMA UART RX Buffer */
uint8_t dma_rx_buffer_[DMA_RX_BUFFER_SIZE];
/** RX Buffer */
uint8_t rx_buffer_[DMA_RX_BUFFER_SIZE];
/** Rx Buffer Pointer. Last index to be read */
uint32_t rx_buffer_end_ = 0;
/** Rx Buffer Pointer. Next index to read. If it is equals to 'rx_buffer_end' -> nothing to read */
uint32_t rx_buffer_0_ = 0;


/* USER CODE END 0 */

/* USART2 init function */

void MX_USART2_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration  
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 DMA Init */
  
  /* USART2_TX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_6, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);

  /* USART2_RX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);

}

/* USER CODE BEGIN 1 */

/**
 * @brief  This enables the USART as DMA UART
 */
void MX_USART2_UART_Enable(

)
{
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)dma_rx_buffer_);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, DMA_RX_BUFFER_SIZE);



  /* Set peripheral address to DMA */
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)&USART2->DR);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)&USART2->DR);

  /* Enable UART DMA request */
  LL_USART_EnableDMAReq_RX(USART2);
  LL_USART_EnableDMAReq_TX(USART2);

  /* Enable LPUART and DMA */
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);

  /* Enable HT & TC interrupts for RX */
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);
  /* Enable HT & TC interrupts for TX */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
  /* Enable UART IDLE interrupt for UART */
  LL_USART_EnableIT_IDLE(USART2);
}


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


/**
 * @brief  Read data in RX-BUFFER
 * @param b - Pointer to the output byte the data
 * @param max - Max size of buff
 * @return  Zero if no data is pending to read, other if one byte is read
 */
uint8_t uart_read_data (
		uint8_t *b
)
{
  uint8_t ret = 0;
  size_t rx_end = rx_buffer_end_;

  if (rx_buffer_0_!=rx_end) {
    /* Copy the byte */
    *b = rx_buffer_[rx_buffer_0_];
    /* Increase the circular buffer index */
    rx_buffer_0_ = (rx_buffer_0_ + 1) % DMA_RX_BUFFER_SIZE;
    /* Update the returned value */
    ret = 1;
  }

  return ret;
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
