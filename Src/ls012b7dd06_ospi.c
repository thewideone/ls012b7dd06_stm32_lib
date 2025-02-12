/*
 * ls012b7dd06_ospi.c
 *
 *  Created on: Feb 11, 2025
 *      Author: szyme
 */

#include "ls012b7dd06_ospi.h"
//#include "tim.h"	// for the halfline timer from the main project (Core/Inc/tim.h)
#include "ls012b7dd06_tim.h"
#include "octospi.h"
#include <stdio.h>	// for sprintf()

OSPI_HandleTypeDef *hlcd_ospi;
OSPI_RegularCmdTypeDef ospi_cmd;
uint8_t lcd_tx_buf_1[OUT_DATA_BUF_SIZE] = {0};

void lcd_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi);

//#ifdef LCD_USE_CUSTOM_CONFIG
//void lcd_OSPI_init(void){
//#else
HAL_StatusTypeDef lcd_OSPI_init(OSPI_HandleTypeDef *hospi){
//#endif
	if(hospi == NULL){
	  hospi = hlcd_ospi;
	  hospi->Instance = OCTOSPI1;
	  hospi->Init.FifoThreshold = 1;
	  hospi->Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
	  hospi->Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
	  hospi->Init.DeviceSize = 32;
	  hospi->Init.ChipSelectHighTime = 1;
	  hospi->Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
	  hospi->Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
	  hospi->Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
	  hospi->Init.ClockPrescaler = 100;
	  hospi->Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
	  hospi->Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
	  hospi->Init.ChipSelectBoundary = 0;
	  hospi->Init.MaxTran = 0;
	  hospi->Init.Refresh = 0;
	  if (HAL_OSPI_Init(hospi) != HAL_OK)
	  {
	    Error_Handler();
	  }
	}
	else {
		hlcd_ospi = hospi;
	}

	HAL_StatusTypeDef ret = HAL_OSPI_RegisterCallback(hospi, HAL_OSPI_TX_CPLT_CB_ID, lcd_OSPI_TxCpltCallback);

	// In a single transmission,
	  // first, an instruction (1<<BSP_PIN) is transmitted to send
	  // the BSP signal pulse
	  // ( apparently, the transmission can not lack both
	  // instruction and address phases at the same time
	  // see body of OSPI_ConfigCmd() used by HAL_OSPI_Command() );
	  // next, the data is sent with two trailing 0x00 bytes
	  // to add two extra dummy clock edges.

	  ospi_cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	  ospi_cmd.Instruction = (1<<6);//(1 << LCD_BSP_Pin);//0x20;			// send the BSP signal pulse
	  ospi_cmd.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES;	// 8-line parallel output
	  ospi_cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;	// transmit the BSP signal on only one edge of the clock
	  //		  .Address = 0x00,
	  //		  .AddressSize = HAL_OSPI_ADDRESS_8_BITS,
	  ospi_cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;	// no address phase
	  ospi_cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	  ospi_cmd.DataMode = HAL_OSPI_DATA_8_LINES;		// 8-line parallel output
	  ospi_cmd.NbData = sizeof(uint8_t)*OUT_DATA_BUF_LINE_W; //OUT_DATA_BUF_SIZE, //18, // size of data to be transmitted, possible range: 1 - 0xFFFFFFFF (1-2^32)
	  ospi_cmd.DataDtrMode = HAL_OSPI_DATA_DTR_ENABLE;	// transmit on both edges of the clock
	  ospi_cmd.DummyCycles = 0; // 0-31
	  ospi_cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
	  ospi_cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;	// send BSP on every transmission

	  return ret;
}

#ifndef LCD_USE_CUSTOM_CONFIG
/**
* @brief OSPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hospi: OSPI handle pointer
* @retval None
*/
void HAL_OSPI_MspInit(OSPI_HandleTypeDef* hospi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  DMA_TriggerConfTypeDef TriggerConfig;
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hospi->Instance==OCTOSPI1)
  {
  /* USER CODE BEGIN OCTOSPI1_MspInit 0 */

  /* USER CODE END OCTOSPI1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_OSPI;
    PeriphClkInit.OspiClockSelection = RCC_OSPICLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_OSPI1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**OCTOSPI1 GPIO Configuration
    PC0     ------> OCTOSPI1_IO7
    PC1     ------> OCTOSPI1_IO4
    PC2     ------> OCTOSPI1_IO5
    PC3     ------> OCTOSPI1_IO6
    PA2     ------> OCTOSPI1_NCS
    PA3     ------> OCTOSPI1_CLK
    PA6     ------> OCTOSPI1_IO3
    PA7     ------> OCTOSPI1_IO2
    PB0     ------> OCTOSPI1_IO1
    PB1     ------> OCTOSPI1_IO0
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_PX_D4_Pin|LCD_PX_D5_Pin|LCD_BSP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2|LCD_PX_CLK_Pin|LCD_PX_D3_Pin|LCD_PX_D2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_PX_D1_Pin|LCD_PX_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* OCTOSPI1 DMA Init */
    /* GPDMA1_REQUEST_OCTOSPI1 Init */
    handle_GPDMA1_Channel12.Instance = GPDMA1_Channel12;
    handle_GPDMA1_Channel12.Init.Request = GPDMA1_REQUEST_OCTOSPI1;
    handle_GPDMA1_Channel12.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel12.Init.Direction = DMA_MEMORY_TO_PERIPH;
    handle_GPDMA1_Channel12.Init.SrcInc = DMA_SINC_INCREMENTED;
    handle_GPDMA1_Channel12.Init.DestInc = DMA_DINC_FIXED;
    handle_GPDMA1_Channel12.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel12.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel12.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA1_Channel12.Init.SrcBurstLength = 1;
    handle_GPDMA1_Channel12.Init.DestBurstLength = 1;
    handle_GPDMA1_Channel12.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA1_Channel12.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel12.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel12) != HAL_OK)
    {
      Error_Handler();
    }

    TriggerConfig.TriggerMode = DMA_TRIGM_BLOCK_TRANSFER;
    TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_RISING;
    TriggerConfig.TriggerSelection = GPDMA1_TRIGGER_TIM15_TRGO;
    if (HAL_DMAEx_ConfigTrigger(&handle_GPDMA1_Channel12, &TriggerConfig) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hospi, hdma, handle_GPDMA1_Channel12);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel12, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* OCTOSPI1 interrupt Init */
    HAL_NVIC_SetPriority(OCTOSPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OCTOSPI1_IRQn);
  /* USER CODE BEGIN OCTOSPI1_MspInit 1 */

  /* USER CODE END OCTOSPI1_MspInit 1 */
  }

}

/**
* @brief OSPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hospi: OSPI handle pointer
* @retval None
*/
void HAL_OSPI_MspDeInit(OSPI_HandleTypeDef* hospi)
{
  if(hospi->Instance==OCTOSPI1)
  {
  /* USER CODE BEGIN OCTOSPI1_MspDeInit 0 */

  /* USER CODE END OCTOSPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_OSPI1_CLK_DISABLE();

    /**OCTOSPI1 GPIO Configuration
    PC0     ------> OCTOSPI1_IO7
    PC1     ------> OCTOSPI1_IO4
    PC2     ------> OCTOSPI1_IO5
    PC3     ------> OCTOSPI1_IO6
    PA2     ------> OCTOSPI1_NCS
    PA3     ------> OCTOSPI1_CLK
    PA6     ------> OCTOSPI1_IO3
    PA7     ------> OCTOSPI1_IO2
    PB0     ------> OCTOSPI1_IO1
    PB1     ------> OCTOSPI1_IO0
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|LCD_PX_D4_Pin|LCD_PX_D5_Pin|LCD_BSP_Pin);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|LCD_PX_CLK_Pin|LCD_PX_D3_Pin|LCD_PX_D2_Pin);

    HAL_GPIO_DeInit(GPIOB, LCD_PX_D1_Pin|LCD_PX_D0_Pin);

    /* OCTOSPI1 DMA DeInit */
    HAL_DMA_DeInit(hospi->hdma);

    /* OCTOSPI1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(OCTOSPI1_IRQn);
  /* USER CODE BEGIN OCTOSPI1_MspDeInit 1 */

  /* USER CODE END OCTOSPI1_MspDeInit 1 */
  }

}

#endif

// Used also in display_frame()
HAL_StatusTypeDef lcd_OSPI_set_cmd_config(void){
	return HAL_OSPI_Command( hlcd_ospi, &ospi_cmd, 2047 );
}

// Used also in display_frame()
HAL_StatusTypeDef lcd_OSPI_transmit_halfline(uint32_t halfline_no){
	return HAL_OSPI_Transmit_DMA( hlcd_ospi, (uint8_t*)lcd_tx_buf_1 + sizeof(lcd_colour_t)*OUT_DATA_BUF_LINE_W*halfline_no );
}

// can't move whole into the driver
// because callbacks can be executed for multiple e.g. timers
// use user callbacks instead!
//void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi){
void lcd_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi){
//	HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
//	sprintf( uart_msg_buf, "OSPI1 tx complete. hlcd_ospi->State = 0x%08X. Transmitting next...\r\n",
//						hlcd_ospi.State );
//	HAL_UART_Transmit( &huart1, uart_msg_buf, sizeof(uart_msg_buf), 10 );

	HAL_StatusTypeDef ret = HAL_OK;

	// Initially 1 because this callback is executed after half-line has already been transmitted
	static uint32_t halfline_no = 1;

	if( halfline_no < 480 ) {
//		HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
		ret = lcd_OSPI_set_cmd_config();

		if( ret != HAL_OK ){
			  sprintf( uart_msg_buf, "Error: HAL_OSPI_Command() status = %s, OSPI error code = 0x%08lX\r\n",
						hal_status_str[ret], hospi->ErrorCode );
			  Error_Handler();
		}

		// Transmit next halfline
//		ret = HAL_OSPI_Transmit_DMA( hospi, (uint8_t*)lcd_tx_buf_1 + sizeof(lcd_colour_t)*OUT_DATA_BUF_LINE_W*halfline_no );
		ret = lcd_OSPI_transmit_halfline(halfline_no);

		if( ret != HAL_OK ){
			sprintf( uart_msg_buf, "Error: HAL_OSPI_Transmit_DMA status = %s, ospi error code = 0x%08lX\r\n",
			hal_status_str[ret], hospi->ErrorCode );
			Error_Handler();
		}
//		HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );

		halfline_no++;
	}
	else {
		halfline_no = 1;
//		HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
		// Clear TIM15 CC1 IF
		__HAL_TIM_CLEAR_FLAG(hlcd_tim_halfline, TIM_FLAG_CC1);
		__HAL_TIM_ENABLE_IT(hlcd_tim_halfline, TIM_IT_CC1);
//		HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
	}

//	HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
}

//__weak void lcd_OSPI_Error_Handler(){
//
//}
