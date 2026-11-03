#ifndef __USER_RADIO_H_
#define __USER_RADIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "lr20xx_radio_lora_types.h"
#include "lr20xx_system_types.h"
#include "lr20xx_system.h"
#include "lr20xx_regmem.h"
#include "lr20xx_hal.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_common_types.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "event.h"
#include "user_uart.h"
#include "string.h"

#define E80_CONTEXT "LR2021"

extern uint8_t rx_buffer_lenght;
extern uint8_t rx_buffer[255];
//extern lr11xx_bsp_tx_cfg_output_params_t  output_params;

void radio_tx_auto( void );
void radio_tx_custom(uint8_t *buffer, uint8_t lenght);
void radio_rx( void );
void radio_sleep(void);
void radio_wakeup(void);
void radio_init( int8_t power, uint32_t freq);
void radio_inits( int8_t power, uint32_t freq, 
									lr20xx_radio_lora_sf_t sf,
									lr20xx_radio_lora_bw_t bw,
									lr20xx_radio_lora_cr_t cr,
								  lr20xx_radio_lora_ppm_t ppm,
								  uint8_t synword);
void radio_init_up(void);
void radio_irq_callback( void );

#ifdef __cplusplus
}
#endif
#endif