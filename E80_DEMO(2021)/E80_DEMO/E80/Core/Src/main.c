/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f1xx_hal_pwr.h"
#include "lr20xx_hal.h"
#include "lr20xx_system.h"
#include "lr20xx_regmem.h"
#include "lr20xx_radio_lora_types.h"
#include "lr20xx_radio_lora.h"
#include "user_uart.h"
#include "event.h"
#include "user_radio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* 串口读取数据缓冲区 */
uint8_t opt_buffer[255] = {0};

/* 默认自定义工作频点 */
uint32_t frequency = 850000000;

/* 工厂测试标志 */
uint8_t factory_test = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void hal_mcu_disable_irq( void )
{
    __disable_irq( );
}

void hal_mcu_enable_irq( void )
{
    __enable_irq( );
}

uint16_t opt_lenght = 0;
lr20xx_system_irq_mask_t radio_irq_test = LR20XX_SYSTEM_IRQ_NONE;
uint8_t scan_test = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	hal_mcu_disable_irq();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
		/* 清除串口接收中断标志 */
	__HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_RXNE);
	
	/* 开启串口接收中断 */
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
	
	/* 串口中断使能 */
	HAL_NVIC_EnableIRQ( USART1_IRQn ); 
	
	/* 串口fifo初始化 */
	uart_fifo_init();
	
	hal_mcu_enable_irq();
	
	/* 射频初始化 */
	radio_init( 0x2C, frequency);
	
  /* 射频进入接收状态 */
	radio_rx();
	
	/* 系统启动打印信息 */
	printf("Radio enter receive\r\n");
	printf("Enter the main program\r\n");
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(event_check( EVENT_UART_RX_DONE ))
		{
			  event_clear(EVENT_UART_RX_DONE);
				/* 读取串口接收内容 */
        uart_read( opt_buffer, &opt_lenght );
			  if((opt_buffer[0] == 0xC1) && (opt_lenght <= 6))
				{
					/* 频率配置 */
					if((opt_buffer[1] == 0x00) && (opt_lenght == 6))
					{				
					    frequency	= 0;
						  frequency = frequency | (uint32_t)( opt_buffer[2] << 24 );
							frequency = frequency | (uint32_t)( opt_buffer[3] << 16 );
							frequency = frequency | (uint32_t)( opt_buffer[4] << 8 );
							frequency = frequency | (uint32_t)( opt_buffer[5] << 0 );
						
						  /* 频率如果大于2.4G 重新配置PA通道及发送参数 */
							if( frequency >= 2400000000 )
							{
									/* LR2021 配置射频工作频点 */
									lr20xx_radio_common_set_rf_freq(E80_CONTEXT, frequency);
									
									const lr20xx_radio_common_pa_cfg_t pa_cfg = {
										.pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
										.pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
										.pa_lf_duty_cycle = 6,
										.pa_lf_slices     = 7,
										.pa_hf_duty_cycle = 16,
									};
									
									lr20xx_radio_common_set_pa_cfg( E80_CONTEXT, &pa_cfg );
									
									lr20xx_radio_common_set_tx_params( E80_CONTEXT, 0x18, LR20XX_RADIO_COMMON_RAMP_304_US );
							}
							/* LR2021 400/900MHz LoRa 配置*/
							else
							{
									/* LR2021 配置射频工作频点 */
									lr20xx_radio_common_set_rf_freq(E80_CONTEXT, frequency);
								
									const lr20xx_radio_common_pa_cfg_t pa_cfg = {
										.pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
										.pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
										.pa_lf_duty_cycle = 7,
										.pa_lf_slices     = 7,
										.pa_hf_duty_cycle = 16,
									};
									
									lr20xx_radio_common_set_pa_cfg( E80_CONTEXT, &pa_cfg );
									
									lr20xx_radio_common_set_tx_params( E80_CONTEXT, 0x2C, LR20XX_RADIO_COMMON_RAMP_304_US );
							}
							
						  radio_rx();
							
						  HAL_UART_Transmit(&huart1,opt_buffer , opt_lenght, 100);
							
					}
					/* 单载波 C1 02 00：stop    C1 02 01：start */
					else if(opt_buffer[1] == 0x02 && (opt_lenght == 3))
					{
							if(opt_buffer[2] == 0x01)
							{
									lr20xx_radio_common_set_tx_test_mode(E80_CONTEXT,LR20XX_RADIO_COMMON_TX_TEST_MODE_CONTINUOUS_WAVE);
								  printf("start tx cw\r\n");
							}
							else if(opt_buffer[2] == 0x00)
							{
									radio_rx();
								  printf("stop tx cw\r\n");
							}
					}
					/* 射频低功耗配置 C1 03 00：退出休眠   C1 03 01：进入休眠 （射频模组7.6uA）*/
					else if(opt_buffer[1] == 0x03 && (opt_lenght == 3))
					{
							if(opt_buffer[2] == 0x01)
							{
								  printf("radio enter sleep\r\n");
									radio_sleep();
								
								  /* 进入休眠后重新配置spi引脚 */
								  HAL_SPI_DeInit(&hspi1);
								  GPIO_InitTypeDef GPIO_InitStruct = {0};
								  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
									GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
									GPIO_InitStruct.Pull = GPIO_PULLUP;
									HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);								
								  /* 用户按照需求选择是否控制单片机进行休眠 具体可以查看stm32f1xx_hal_pwr.c描述
								     HAL_PWR_EnterSLEEPMode：单片机进入深度休眠
										 HAL_PWR_EnterSTOPMode ：单片机进入停止模式 */
								  HAL_PWR_EnterSLEEPMode(1, PWR_SLEEPENTRY_WFI);
                  //HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);				 
							}
							else if(opt_buffer[2] == 0x00)
							{
								  /* RF复位唤醒 */
								  MX_SPI1_Init();
								  radio_init( 22, frequency);
								  radio_rx();
								  printf("radio exit sleep\r\n");
							}
					}
					/* 自动发送数据 C1 C1 C1 */
					else if(opt_buffer[1] == 0xC1 && opt_buffer[2] == 0xC1 && (opt_lenght == 3))
					{ 
							radio_tx_auto();
					}
				}
				/* 一次性配置射频参数接口 */
				else if(opt_buffer[0] == 0xC2 && opt_lenght == 12)
				{
					  //lr11xx_pa_type_t pa_types = opt_buffer[1];
					  uint8_t powers              = opt_buffer[2];
					  lr20xx_radio_lora_sf_t sf   = opt_buffer[3];
					  lr20xx_radio_lora_bw_t bw   = opt_buffer[4];
					  lr20xx_radio_lora_cr_t cr   = opt_buffer[5];
					  lr20xx_radio_lora_ppm_t ppm = opt_buffer[6];
					  uint8_t synword             = opt_buffer[7];
					  frequency                   = 0;
					  frequency = frequency | (uint32_t)( opt_buffer[8] << 24 );
						frequency = frequency | (uint32_t)( opt_buffer[9] << 16 );
						frequency = frequency | (uint32_t)( opt_buffer[10] << 8 );
						frequency = frequency | (uint32_t)( opt_buffer[11] << 0 );
						radio_inits( powers, frequency, sf, bw, cr, ppm, synword);
					  radio_rx();
				}
				/* factory test mode */
				else if((opt_buffer[0] == 0xC0) && (opt_buffer[1] == 0x00) && (opt_buffer[2] == 0x01) && (opt_lenght == 3))
				{
					  opt_buffer[0] = 0xC0;
					  opt_buffer[1] = 0x01;
					  opt_buffer[2] = 0x01;
						HAL_UART_Transmit(&huart1, (const uint8_t *)opt_buffer, opt_lenght, 100);
					  factory_test = true;
				}
				/* 低空速距离测试 C3 C3 00 + 频率 Sub-G 0.292kbps    C3 C3 01 + 频率 2.4G 0.476kbps*/
				else if((opt_buffer[0] == 0xC3) && (opt_buffer[1] == 0xC3) && (opt_lenght == 7))
				{
					  frequency = 0;
					  frequency = frequency | (uint32_t)( opt_buffer[3] << 24 );
						frequency = frequency | (uint32_t)( opt_buffer[4] << 16 );
						frequency = frequency | (uint32_t)( opt_buffer[5] << 8 );
						frequency = frequency | (uint32_t)( opt_buffer[6] << 0 );
					
					  if(opt_buffer[2] == 0x00)
						{
							if(frequency > 950000000)
								frequency = 950000000;
							radio_inits( 0x2C, frequency, LR20XX_RADIO_LORA_SF12, 
													 LR20XX_RADIO_LORA_BW_125, LR20XX_RADIO_LORA_CR_4_5, 1, 0x12);
						}
						else if(opt_buffer[2] == 0x01)
						{
							if(frequency < 2400000000)
								frequency = 2400000000;
							radio_inits( 0x18, frequency, LR20XX_RADIO_LORA_SF12, 
													 LR20XX_RADIO_LORA_BW_203, LR20XX_RADIO_LORA_CR_4_5, 1, 0x12);
						}
						/* 2.4K 对比测试 */
						else if(opt_buffer[2] == 0x02)
						{
							radio_inits(0x2C, frequency, LR20XX_RADIO_LORA_SF11, 
													 LR20XX_RADIO_LORA_BW_500, LR20XX_RADIO_LORA_CR_4_5, 1, 0x12);
						}
						radio_rx();
						HAL_UART_Transmit(&huart1, (const uint8_t *)opt_buffer, opt_lenght, 100);
				}
				/* 灵敏度测试 */
				else if((opt_buffer[0] == 0xC4) && (opt_buffer[1] == 0xC4) && (opt_lenght == 7))
				{
						frequency = 0;
					  frequency = frequency | (uint32_t)( opt_buffer[3] << 24 );
						frequency = frequency | (uint32_t)( opt_buffer[4] << 16 );
						frequency = frequency | (uint32_t)( opt_buffer[5] << 8 );
						frequency = frequency | (uint32_t)( opt_buffer[6] << 0 );
					  
					  if(opt_buffer[2] == 0x00)
						{
							if(frequency > 950000000)
								frequency = 950000000;
							radio_inits(0x2C, frequency, LR20XX_RADIO_LORA_SF9, 
													 LR20XX_RADIO_LORA_BW_125, LR20XX_RADIO_LORA_CR_4_5, 0, 0x12);
						}
						else if(opt_buffer[2] == 0x01)
						{
							if(frequency < 2400000000)
								frequency = 2400000000;
							radio_inits(0x18, frequency, LR20XX_RADIO_LORA_SF9, 
													 LR20XX_RADIO_LORA_BW_125, LR20XX_RADIO_LORA_CR_4_5, 0, 0x12);
						}
						radio_rx();
						HAL_UART_Transmit(&huart1, (const uint8_t *)opt_buffer, opt_lenght, 100);
				}
				/* 如果串口接收数据不满足上述指令配置 直接进行无线传输 */
				else
				{
					radio_tx_custom(opt_buffer,opt_lenght);
					memset(opt_buffer,0,sizeof(opt_buffer));
				}
		}
		/* 射频接收数据完成 */
		if(event_check( EVENT_RADIO_RX_DONE ))
		{
				event_clear(EVENT_RADIO_RX_DONE);
			
				/* 未检测到factory test 标志 直接打印无线接收到的数据 */
			  if(!factory_test)
				{
					 /* 阻塞式打印射频接收数据 */
					 HAL_UART_Transmit(&huart1, (const uint8_t *)rx_buffer, rx_buffer_lenght, 100);
					 memset(rx_buffer,0,sizeof(rx_buffer));
				}
				/* 检测到factory test 标志 直接把无线接收到的数据通过无线回环给发送方 */
				else
				{
					 HAL_Delay(200);
					 radio_tx_custom(rx_buffer, rx_buffer_lenght);
					 memset(rx_buffer,0,sizeof(rx_buffer));
				}
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E80_NRST_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_SET);
	
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, E80_DIO8_Pin|E80_DIO9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : E80_BUSY_Pin */
  GPIO_InitStruct.Pin = E80_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(E80_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_NSS_Pin */
  GPIO_InitStruct.Pin = RADIO_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADIO_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E80_NRST_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = E80_NRST_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : E80_DIO9_Pin E80_DIO8_Pin */
  GPIO_InitStruct.Pin = E80_DIO8_Pin | E80_DIO9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : E80_DIO7_Pin KEY2_Pin KEY1_Pin */
  GPIO_InitStruct.Pin = KEY2_Pin|KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
    // 将字符发送到串口1，超时时间设为最大值
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
