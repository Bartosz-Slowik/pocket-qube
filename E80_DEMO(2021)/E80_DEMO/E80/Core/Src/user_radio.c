#include "user_radio.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "event.h"

#define SYNC_WORD_NO_RADIO  0x34

/* 射频参数配置 */
lr20xx_radio_lora_pkt_params_t radio_pkt_params = 
{
	.preamble_len_in_symb = 8,
	.pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT,
	.pld_len_in_bytes = 255,
	.crc = true,
  .iq = false
};

lr20xx_radio_common_pa_cfg_t pa_cfgs = {
	.pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
  .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
  .pa_lf_duty_cycle = 7,
  .pa_lf_slices     = 7,
  .pa_hf_duty_cycle = 16,
};

lr20xx_system_sleep_cfg_t sleep_cfgs = {
  .is_ram_retention_enabled = true,
	.is_clk_32k_enabled = false,
};

/* LoRa 参数配置 */
lr20xx_radio_lora_mod_params_t mod_params = 
{
	.bw  = LR20XX_RADIO_LORA_BW_125,
	.cr  = LR20XX_RADIO_LORA_CR_4_5,
	.sf  = LR20XX_RADIO_LORA_SF8,
  .ppm = LR20XX_RADIO_LORA_NO_PPM,
};

/* 自动发送数据定义 */
uint8_t tx_buffer[] = {0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9};
	
/* 接收数据缓冲区 */
uint8_t rx_buffer[255] = {0};
uint8_t rx_buffer_lenght = 0;
lr20xx_system_version_t version = {0};
lr20xx_radio_common_pkt_type_t pkt_type = LR20XX_RADIO_COMMON_PKT_TYPE_FSK;
uint16_t errors = 0;
uint8_t TXCO = true;
lr20xx_system_irq_mask_t radio_irq = LR20XX_SYSTEM_IRQ_NONE;

/**
  * @brief GPIO中断服务函数
  * @param GPIO_Pin 引脚序号.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == E80_DIO8_Pin)
	{
		radio_irq_callback();
	}
}

/**
  * @brief DIO8射频回调函数（射频接收及发送完成中断服务函数）
  * @param GPIO_Pin 引脚序号.
  * @retval None
  */
void radio_irq_callback( void )
{
	/* 获取irq状态 */
	if(lr20xx_system_get_and_clear_irq_status(E80_CONTEXT ,&radio_irq) != LR20XX_STATUS_OK )
	{
		//printf("get_irq_status failed\r\n");
	}
	
	/* LR2021 超时判断 */
	if(( radio_irq & LR20XX_SYSTEM_IRQ_TIMEOUT ) == LR20XX_SYSTEM_IRQ_TIMEOUT)
	{
		//printf("TIMEOUT\r\n");
		
		/* LR2021 进入接收 */
		radio_rx();
	}
	else if(radio_irq & LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR)
	{
		//printf("HEADER_ERR\r\n"); 
		
		radio_rx();
	}
	else if(radio_irq & LR20XX_SYSTEM_IRQ_CRC_ERROR)
	{
		//printf("CRC_ERR\r\n");
		
		radio_rx();
	}
  /* LR2021 接收完成 */
	else if((radio_irq & LR20XX_SYSTEM_IRQ_RX_DONE) == LR20XX_SYSTEM_IRQ_RX_DONE)
  {		
		 lr20xx_radio_lora_packet_status_t rx_buffer_status;
		
		 /* LR2021 获取接收数据包状态及参数 */
		 lr20xx_radio_lora_get_packet_status( E80_CONTEXT, &rx_buffer_status );
		
		 rx_buffer_lenght = rx_buffer_status.packet_length_bytes;
		
		 /* LR2021 读取FIFO 接收到的数据到 rx_buffer缓冲区*/
		 lr20xx_radio_fifo_read_rx( E80_CONTEXT, rx_buffer, rx_buffer_status.packet_length_bytes );
		
		 /* 产生有一个射频接收完成的事件 由主程序处理 */
		 event_set(EVENT_RADIO_RX_DONE); 
		
		 /* 调试接口 */
		 //HAL_UART_Transmit(&huart1, rx_buffer , rx_buffer_status.pld_len_in_bytes, 100);
		 //memset(rx_buffer,0,sizeof(rx_buffer));
		 //printf("RX DONE\r\n");
		
		 /* LR2021 进入接收 */
		 radio_rx();
  }
	/* LR1121 发射完成 */
	else if((radio_irq & LR20XX_SYSTEM_IRQ_TX_DONE) == LR20XX_SYSTEM_IRQ_TX_DONE)
	{  
		/* 调试接口 */
		//printf("TX DONE\r\n");
		
		/* LR2021 进入接收 */
		radio_rx();
	}
	/* LR2021 其他中断事件 */
	else
	{
		radio_rx();
	}
}

/**
  * @brief 射频初始化
  * @param lr11xx_pa_type_t pa_type：PA通道选择
  * @param int8_t power： 功率配置
  * @param uint32_t freq：频率配置
  * @retval None
  */
void radio_init( int8_t power, uint32_t freq)
{
	/* LR2021 reset */
  lr20xx_hal_reset(E80_CONTEXT);
	
	/* LR2021 获取错误状态(上电先清除所有错误状态) */
	lr20xx_system_get_errors( E80_CONTEXT, &errors );
	
	lr20xx_system_clear_errors(E80_CONTEXT);
	
	/* LR2021 有源晶振 */
	if( TXCO == true )
	{
			/* LR2021 配置模式DC-DC */
			lr20xx_system_set_reg_mode(E80_CONTEXT ,LR20XX_SYSTEM_REG_MODE_DCDC);
	
			/* LR2021 配置TCXO供电电压和检测超时（晶振要求2ms启动时间） */
			lr20xx_system_set_tcxo_mode( E80_CONTEXT, LR20XX_SYSTEM_TCXO_CTRL_2_2V , 64000 ); 
	}
	/* LR2021 无源晶振 */
	else
	{
		  /* LR2021 配置模式Standby */
			lr20xx_system_set_standby_mode(E80_CONTEXT,LR20XX_SYSTEM_STANDBY_MODE_RC);
	
		  /* LR2021 配置模式负载电容 */
			lr20xx_system_configure_xosc(E80_CONTEXT,0x1C,0x1C,0xFF);
	}

	/* LR2021 配置低频(LF)时钟 */
	lr20xx_system_cfg_lfclk(E80_CONTEXT,LR20XX_SYSTEM_LFCLK_RC);
	
	/* LR2021 校准 */
	lr20xx_system_calibrate(E80_CONTEXT,0x7F);
	
	/* LR2021 获取错误状态 */
	lr20xx_system_get_errors( E80_CONTEXT, &errors );
	
	if( errors != 0 )
  {
		  lr20xx_system_clear_errors(E80_CONTEXT);
			printf("TXCO init:lr20xx_system_get_errors:%d\r\n",errors);
  }

	lr20xx_system_set_dio_function( E80_CONTEXT, LR20XX_SYSTEM_DIO_8, LR20XX_SYSTEM_DIO_FUNC_IRQ, LR20XX_SYSTEM_DIO_DRIVE_PULL_DOWN );
	
	/* LR2021 获取版本号 */
	lr20xx_system_get_version(E80_CONTEXT,&version);
	
	printf("major:%x minor:%x\r\n",version.major,version.minor);
	
	/* LR2021 配置数据包类型LORA */
	lr20xx_radio_common_set_pkt_type( E80_CONTEXT, LR20XX_RADIO_COMMON_PKT_TYPE_LORA );
	
  /* LR2021 配置射频数据包参数 */
	lr20xx_radio_lora_set_packet_params( E80_CONTEXT , &radio_pkt_params );
	
	/* LR2021 配置射频同步字参数 */
	lr20xx_radio_lora_set_syncword(E80_CONTEXT,SYNC_WORD_NO_RADIO);
	
	/* LR2021 配置射频数据调制参数 */
	lr20xx_radio_lora_set_modulation_params(E80_CONTEXT, &mod_params);
	
	/* LR2021 2.4G LoRa 配置*/
	if( freq >= 2400000000 )
	{
			/* LR2021 配置射频工作频点 */
		  lr20xx_radio_common_set_rf_freq(E80_CONTEXT, freq);
		  
		  const lr20xx_radio_common_pa_cfg_t pa_cfg = {
        .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
        .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
        .pa_lf_duty_cycle = 6,
        .pa_lf_slices     = 7,
        .pa_hf_duty_cycle = 16,
      };
			
			/* LR2021 配置接收链路 */
      lr20xx_radio_common_set_rx_path( E80_CONTEXT, LR20XX_RADIO_COMMON_RX_PATH_HF, LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE );
			
			lr20xx_radio_common_set_pa_cfg( E80_CONTEXT, &pa_cfg );
			
			if(power > 0x18)
			{
			   power = 0x18;
			}
			
			lr20xx_radio_common_set_tx_params( E80_CONTEXT, power, LR20XX_RADIO_COMMON_RAMP_304_US );
	}
	/* LR2021 400/900MHz LoRa 配置*/
	else
  {
			/* LR2021 配置射频工作频点 */
		  lr20xx_radio_common_set_rf_freq(E80_CONTEXT, freq);
		
		  if((freq > 400000000) & (freq <550000000))
			{
					 pa_cfgs.pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF;
					 pa_cfgs.pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM;
				   pa_cfgs.pa_lf_duty_cycle = 7;
				   pa_cfgs.pa_lf_slices     = 7;
				   pa_cfgs.pa_hf_duty_cycle = 16;
			}
			else
			{
					 pa_cfgs.pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF;
					 pa_cfgs.pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM;
				   pa_cfgs.pa_lf_duty_cycle = 7;
				   pa_cfgs.pa_lf_slices     = 6;
				   pa_cfgs.pa_hf_duty_cycle = 16;
			}
		
			/* LR2021 配置接收链路 */
      lr20xx_radio_common_set_rx_path( E80_CONTEXT, LR20XX_RADIO_COMMON_RX_PATH_LF, LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE );
			
			lr20xx_radio_common_set_pa_cfg( E80_CONTEXT, &pa_cfgs );
			
			if(power > 0x2C)
			{
					power = 0x2C;
			}
			
			lr20xx_radio_common_set_tx_params( E80_CONTEXT, power, LR20XX_RADIO_COMMON_RAMP_304_US );
	}
	
	/* LR2021 模式自动退回配置 */
	lr20xx_radio_common_set_rx_tx_fallback_mode(E80_CONTEXT, LR20XX_RADIO_FALLBACK_STDBY_RC);
}

/**
  * @brief 射频快速初始化（备用）
  * @param lr11xx_pa_type_t pa_type：PA通道选择
  * @param int8_t power： 功率配置
  * @param uint32_t freq：频率配置
  * @param lr11xx_radio_lora_sf_t sf：  LoRa 扩频因子配置
  * @param lr11xx_radio_lora_bw_t bw：  LoRa 带宽配置
  * @param lr11xx_radio_lora_cr_t cr：  LoRa CRC配置
  * @param lr20xx_radio_lora_ppm_t ppm：LoRa 低空速优化配置
  * @param uint8_t ldros：低空速优化开关配置
  * @param uint8_t synword：LoRa 同步字配置
  * @retval None
  */
void radio_inits( int8_t power, uint32_t freq, 
									lr20xx_radio_lora_sf_t sf,
									lr20xx_radio_lora_bw_t bw,
									lr20xx_radio_lora_cr_t cr,
								  lr20xx_radio_lora_ppm_t ppm,
								  uint8_t synword)
{
	uint16_t errors;
	
	lr20xx_hal_reset(E80_CONTEXT);
	
	/* LR2021 获取错误状态(上电先清除所有错误状态) */
	lr20xx_system_get_errors( E80_CONTEXT, &errors );
	
	lr20xx_system_clear_errors(E80_CONTEXT);
	
	/* LR2021 有源晶振 */
	if( TXCO == true )
	{
			/* LR2021 配置模式DC-DC */
			lr20xx_system_set_reg_mode(E80_CONTEXT ,LR20XX_SYSTEM_REG_MODE_DCDC);
	
			/* LR2021 配置TCXO供电电压和检测超时（晶振要求2ms启动时间） */
			lr20xx_system_set_tcxo_mode( E80_CONTEXT, LR20XX_SYSTEM_TCXO_CTRL_2_2V , 64000 ); 
	}
	/* LR2021 无源晶振 */
	else
	{
		  /* LR2021 配置模式Standby */
			lr20xx_system_set_standby_mode(E80_CONTEXT,LR20XX_SYSTEM_STANDBY_MODE_RC);
	
		  /* LR2021 配置模式负载电容 */
			lr20xx_system_configure_xosc(E80_CONTEXT,0x1C,0x1C,0xFF);
	}

	/* LR2021 配置低频(LF)时钟 */
	lr20xx_system_cfg_lfclk(E80_CONTEXT,LR20XX_SYSTEM_LFCLK_RC);
	
	/* LR2021 校准 */
	lr20xx_system_calibrate(E80_CONTEXT,0x7F);
	
	/* LR2021 获取错误状态 */
	lr20xx_system_get_errors( E80_CONTEXT, &errors );
	
	if( errors != 0 )
  {
		  lr20xx_system_clear_errors(E80_CONTEXT);
			printf("TXCO init:lr20xx_system_get_errors:%d\r\n",errors);
  }
	
	lr20xx_system_set_dio_function( E80_CONTEXT, LR20XX_SYSTEM_DIO_8, LR20XX_SYSTEM_DIO_FUNC_IRQ, LR20XX_SYSTEM_DIO_DRIVE_PULL_DOWN );
	
	/* LR2021 获取版本号 */
	lr20xx_system_get_version(E80_CONTEXT,&version);
	
	printf("LR2021 Version major:%x minor:%x\r\n",version.major,version.minor);
	
	/* LR2021 配置数据包类型LORA */
	lr20xx_radio_common_set_pkt_type( E80_CONTEXT, LR20XX_RADIO_COMMON_PKT_TYPE_LORA );
	
  /* LR2021 配置射频数据包参数 */
	lr20xx_radio_lora_set_packet_params( E80_CONTEXT , &radio_pkt_params );
	
	/* LR2021 配置射频同步字参数 */
	lr20xx_radio_lora_set_syncword(E80_CONTEXT,synword);
	
	mod_params.bw  = bw;
	mod_params.cr  = cr;
	mod_params.sf  = sf;
	mod_params.ppm = ppm; //低空速优化
	
	/* LR2021 配置射频数据调制参数 */
	lr20xx_radio_lora_set_modulation_params(E80_CONTEXT, &mod_params);
	
	/* LR2021 2.4G LoRa 配置*/
	if( freq >= 2400000000 )
	{
			/* LR2021 配置射频工作频点 */
		  lr20xx_radio_common_set_rf_freq(E80_CONTEXT, freq);
		  
		  const lr20xx_radio_common_pa_cfg_t pa_cfg = {
        .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
        .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
        .pa_lf_duty_cycle = 6,
        .pa_lf_slices     = 7,
        .pa_hf_duty_cycle = 16,
      };
			
			/* LR2021 配置接收链路 */
      lr20xx_radio_common_set_rx_path( E80_CONTEXT, LR20XX_RADIO_COMMON_RX_PATH_HF, LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE );
			
			lr20xx_radio_common_set_pa_cfg( E80_CONTEXT, &pa_cfg );
			
			if(power > 0x18)
			{
					power = 0x18;
			}
			
			lr20xx_radio_common_set_tx_params( E80_CONTEXT, power, LR20XX_RADIO_COMMON_RAMP_304_US );
	}
	/* LR2021 400/900MHz LoRa 配置*/
	else
  {
			/* LR2021 配置射频工作频点 */
		  lr20xx_radio_common_set_rf_freq(E80_CONTEXT, freq);
		
		  if((freq > 400000000) & (freq <550000000))
			{
					 pa_cfgs.pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF;
					 pa_cfgs.pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM;
				   pa_cfgs.pa_lf_duty_cycle = 7;
				   pa_cfgs.pa_lf_slices     = 7;
				   pa_cfgs.pa_hf_duty_cycle = 16;
			}
			else
			{
					 pa_cfgs.pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF;
					 pa_cfgs.pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM;
				   pa_cfgs.pa_lf_duty_cycle = 7;
				   pa_cfgs.pa_lf_slices     = 6;
				   pa_cfgs.pa_hf_duty_cycle = 16;
			}
			
			/* LR2021 配置接收链路 */
      lr20xx_radio_common_set_rx_path( E80_CONTEXT, LR20XX_RADIO_COMMON_RX_PATH_LF, LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE );
			
			lr20xx_radio_common_set_pa_cfg( E80_CONTEXT, &pa_cfgs );
			
			if(power > 0x2C)
			{
					power = 0x2C;
			}
			
			lr20xx_radio_common_set_tx_params( E80_CONTEXT, power, LR20XX_RADIO_COMMON_RAMP_304_US );
	}
	
	/* LR2021 模式自动退回配置 */
	lr20xx_radio_common_set_rx_tx_fallback_mode(E80_CONTEXT, LR20XX_RADIO_FALLBACK_STDBY_RC);
}

/**
  * @brief 射频进入接收
  * @param None
  * @retval None
  */
void radio_rx( void )
{
	/* LR2021 配置射频数据包长度 */
	radio_pkt_params.pld_len_in_bytes = 255;
	
	/* LR2021 配置射频数据长度 */
	lr20xx_radio_lora_set_packet_params( E80_CONTEXT , &radio_pkt_params );
	
	/* LR2021 配置DIO9中断 */
	lr20xx_system_set_dio_irq_cfg(E80_CONTEXT, LR20XX_SYSTEM_DIO_8,
																						 LR20XX_SYSTEM_IRQ_RX_DONE |		
																						 LR20XX_SYSTEM_IRQ_CRC_ERROR );
	
	/* LR2021 清除DIO9中断状态 */
	lr20xx_system_clear_irq_status(E80_CONTEXT, LR20XX_SYSTEM_IRQ_ALL_MASK);
	
	/* LR2021 进入接收模式 */
	lr20xx_radio_common_set_rx(E80_CONTEXT, 0);
}

/**
  * @brief 射频发送
  * @param None
  * @retval None
  */
void radio_tx_auto(void)
{
	/* LR2021 配置DIO9中断 */
	lr20xx_system_set_dio_irq_cfg(E80_CONTEXT, LR20XX_SYSTEM_DIO_8, LR20XX_SYSTEM_IRQ_TX_DONE);
	
	/* LR2021 清除DIO9中断状态 */
	lr20xx_system_clear_irq_status( E80_CONTEXT, LR20XX_SYSTEM_IRQ_ALL_MASK);
	
	/* LR2021 获取需要发射数据长度 */
	radio_pkt_params.pld_len_in_bytes = sizeof(tx_buffer);
	
	/* LR2021 配置发射数据长度 */
	lr20xx_radio_lora_set_packet_params( E80_CONTEXT, &radio_pkt_params );
	
	/* 写入数据到 LR2021 到 FIFO */
	lr20xx_radio_fifo_write_tx( E80_CONTEXT , tx_buffer, sizeof(tx_buffer));
	
	/* LR2021 开始发送 */
	lr20xx_radio_common_set_tx(E80_CONTEXT,0);
}

/**
  * @brief 射频自定义发送API
  * @param uint8_t *buffer：指向发送数据
  * @param uint8_t lenght： 数据长度
  * @retval None
  */
void radio_tx_custom(uint8_t *buffer, uint8_t lenght)
{
	/* LR2021 配置发射数据长度 */
	lr20xx_system_set_dio_irq_cfg(E80_CONTEXT, LR20XX_SYSTEM_DIO_8, LR20XX_SYSTEM_IRQ_TX_DONE);
	
	/* LR2021 清除DIO9中断状态 */
	lr20xx_system_clear_irq_status(E80_CONTEXT, LR20XX_SYSTEM_IRQ_ALL_MASK);
	
	/* LR2021 获取需要发射数据长度 */
	radio_pkt_params.pld_len_in_bytes = lenght;
	
	/* LR2021 配置发射数据长度 */
	lr20xx_radio_lora_set_packet_params( E80_CONTEXT, &radio_pkt_params );
	
	/* LR2021 配置发射数据长度 */
	lr20xx_radio_fifo_write_tx( E80_CONTEXT , buffer, lenght);
	
	/* LR2021 开始发送 */
	lr20xx_radio_common_set_tx(E80_CONTEXT,0);
}

/**
  * @brief 射频进入低功耗
  * @param None
  * @retval None
  */
void radio_sleep(void)
{
	lr20xx_system_set_sleep_mode( E80_CONTEXT, &sleep_cfgs, 0 );
}

/**
  * @brief 射频唤醒
  * @param None
  * @retval None
  */
void radio_wakeup(void)
{
	/* LR2021 休眠唤醒 */
	lr20xx_hal_wakeup(E80_CONTEXT);
}
