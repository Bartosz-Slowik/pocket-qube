/**
 * @file      ral_lr20xx_bsp.c
 *
 * @brief     HAL implementation for LR20xx radio chip.
 *
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stddef.h>

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "lr20xx_hal.h"
#include "stm32f1xx_hal.h"
//#include "smtc_hal_gpio.h"
//#include "smtc_hal_spi.h"
//#include "smtc_hal_mcu.h"
//#include "modem_pinout.h"
#include "main.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum
{
    RADIO_SLEEP,
    RADIO_AWAKE
} radio_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static volatile radio_mode_t radio_mode = RADIO_AWAKE;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
extern SPI_HandleTypeDef hspi1;

/**
 * @brief Wait until radio busy pin returns to 0
 */
static void lr20xx_hal_wait_on_busy( void );

/**
 * @brief Check if device is ready to receive spi transaction.
 * @remark If the device is in sleep mode, it will awake it and wait until it is ready
 */
static void lr20xx_hal_check_device_ready( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_hal_status_t lr20xx_hal_reset( const void* radio )
{
    //hal_gpio_set_value( RADIO_NRST, 0 );
	  HAL_GPIO_WritePin( E80_NRST_GPIO_Port, E80_NRST_Pin, GPIO_PIN_RESET );
	
    // wait for 1ms
    //hal_mcu_wait_us( 1000 );
	  HAL_Delay( 10 );
	
    //hal_gpio_set_value( RADIO_NRST, 1 );
    HAL_GPIO_WritePin( E80_NRST_GPIO_Port, E80_NRST_Pin, GPIO_PIN_SET );
	
	  radio_mode = RADIO_AWAKE;
	
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_wakeup( const void* radio )
{
    // Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS
    //hal_gpio_set_value( RADIO_NSS, 0 );
	  HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_RESET );
    // wait for 1ms
    //hal_mcu_wait_us( 1000 );
	  HAL_Delay( 10 );
	
    //hal_gpio_set_value( RADIO_NSS, 1 );
	  HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET );
	
    radio_mode = RADIO_AWAKE;
	
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_read( const void* radio, const uint8_t* cbuffer, const uint16_t cbuffer_length,
                                     uint8_t* rbuffer, const uint16_t rbuffer_length )
{
    uint8_t dummy_bytes[2] = { 0x00, 0x00 };

    lr20xx_hal_check_device_ready( );

    // Put NSS low to start spi transaction
		HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_RESET );

		if(HAL_SPI_Receive( &hspi1, ( uint8_t* ) cbuffer, cbuffer_length, 100 ) != HAL_OK)
		{
			return LR20XX_HAL_STATUS_ERROR;
		}

    HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET );
		
    if( rbuffer_length > 0 )
    {
        lr20xx_hal_wait_on_busy( );    
			  HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_RESET );
			
        // Send dummy bytes
			  HAL_SPI_Transmit( &hspi1, dummy_bytes, 2, 100 );
			  
				
				if(HAL_SPI_Receive( &hspi1, rbuffer, rbuffer_length, 100 ) != HAL_OK)
				{
					 return LR20XX_HAL_STATUS_ERROR;
				}

        // Put NSS high as the spi transaction is finished
        HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET );
    }

    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_write( const void* radio, const uint8_t* cbuffer, const uint16_t cbuffer_length,
                                      const uint8_t* cdata, const uint16_t cdata_length )
{
	  HAL_StatusTypeDef status = HAL_ERROR;
	
    lr20xx_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_RESET );

    // Send command
    if(HAL_SPI_Transmit( &hspi1, ( uint8_t* ) cbuffer, cbuffer_length, 100 ) != HAL_OK)
		{
			return LR20XX_HAL_STATUS_ERROR;
		}
    // Send data
    if(cdata_length > 0)
		{
			if(HAL_SPI_Transmit( &hspi1, ( uint8_t* ) cdata, cdata_length, 100 ) != HAL_OK)
			{
				return LR20XX_HAL_STATUS_ERROR;
			}
		}

    // Put NSS high as the spi transaction is finished
    HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET );

    // Check if command sent is a sleep command LR20XX_SYSTEM_SET_SLEEP_MODE_OC = 0x0127 and save context
    if( ( cbuffer[0] == 0x01 ) && ( cbuffer[1] == 0x1B ) )
    {
        radio_mode = RADIO_SLEEP;

        // add a incompressible delay to prevent trying to wake the radio before it is full asleep
        HAL_Delay( 1 );
    }
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read( const void* radio, uint8_t* data, const uint16_t data_length )
{
	  HAL_StatusTypeDef status = HAL_ERROR;
    lr20xx_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_RESET );

    status = HAL_SPI_Receive( &hspi1, data, data_length, 100 );

    // Put NSS high as the spi transaction is finished
    HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET );

    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read_fifo( const void* radio, const uint8_t* command,
                                                 const uint16_t command_length, uint8_t* data,
                                                 const uint16_t data_length )
{
    lr20xx_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_RESET );

    // Send command
    if(HAL_SPI_Receive( &hspi1, command, command_length, 100 ) != HAL_OK)
	  {
				return LR20XX_HAL_STATUS_ERROR;
		}

    // Send data
		if(HAL_SPI_Receive( &hspi1, data, data_length, 100 ) != HAL_OK)
		{
				 return LR20XX_HAL_STATUS_ERROR;
		}

    // Put NSS high as the spi transaction is finished
    HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET );

    return LR20XX_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lr20xx_hal_wait_on_busy( void )
{
    while( HAL_GPIO_ReadPin( E80_BUSY_GPIO_Port, E80_BUSY_Pin ));
}

static void lr20xx_hal_check_device_ready( void )
{
    if( radio_mode != RADIO_SLEEP )
    {
        lr20xx_hal_wait_on_busy( );
    }
    else
    {
        // Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS
        HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_RESET );
			  HAL_Delay( 5 );
        HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET );
        lr20xx_hal_wait_on_busy( );
        radio_mode = RADIO_AWAKE;
    }
}




/* --- EOF ------------------------------------------------------------------ */
