/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_smartdma.h"
#include "fsl_power.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Whether the SW button is pressed */
volatile bool g_ButtonPress = false;
smartdma_keyscan_4x4_param_t smartdmaParam;                  /*!< SMARTDMA function parameters. */
volatile uint8_t g_samrtdma_stack[32];
volatile uint32_t g_keyscan_gpio_register[8] = {
		(uint32_t)& GPIO->W[0][13],/*ROW1, P0_13,Pin Data Register*/
		(uint32_t)& GPIO->W[0][14],/*ROW2, P0_14,Pin Data Register*/
		(uint32_t)& GPIO->W[1][19],/*ROW3, P1_19,Pin Data Register*/
		(uint32_t)& GPIO->W[1][28],/*ROW4, P1_28,Pin Data Register*/
		(uint32_t)& GPIO->W[0][6], /*COL1, P0_6, Pin Data Register */
		(uint32_t)& GPIO->W[0][2], /*COL2, P0_2, Pin Data Register*/
		(uint32_t)& GPIO->W[0][3], /*COL3, P0_3, Pin Data Register*/
		(uint32_t)& GPIO->W[0][4], /*COL4, P0_4, Pin Data Register*/
};
volatile uint32_t KeyValue[8]={0,0,0,0,0};
volatile uint32_t g_keyscan_interval = 1000;
volatile uint32_t g_keyscan_complete_flag=0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
static void SmartDMA_keyscan_callback(void *param){
	g_keyscan_complete_flag = 1;
	SMARTDMA_AccessShareRAM(1) ;

	if(KeyValue[0]& 0x01){PRINTF("Button 1  is pressed \r\n");}
	if(KeyValue[0]& 0x02){PRINTF("Button 4  is pressed \r\n");}
	if(KeyValue[0]& 0x04){PRINTF("Button 7  is pressed \r\n");}
	if(KeyValue[0]& 0x08){PRINTF("Button 0  is pressed \r\n");}

	if(KeyValue[1]& 0x01){PRINTF("Button 2  is pressed \r\n");}
	if(KeyValue[1]& 0x02){PRINTF("Button 5  is pressed \r\n");}
	if(KeyValue[1]& 0x04){PRINTF("Button 8  is pressed \r\n");}
	if(KeyValue[1]& 0x08){PRINTF("Button F  is pressed \r\n");}

	if(KeyValue[2]& 0x01){PRINTF("Button 3  is pressed \r\n");}
	if(KeyValue[2]& 0x02){PRINTF("Button 6  is pressed \r\n");}
	if(KeyValue[2]& 0x04){PRINTF("Button 9  is pressed \r\n");}
	if(KeyValue[2]& 0x08){PRINTF("Button E  is pressed \r\n");}

	if(KeyValue[3]& 0x01){PRINTF("Button A  is pressed \r\n");}
	if(KeyValue[3]& 0x02){PRINTF("Button B  is pressed \r\n");}
	if(KeyValue[3]& 0x04){PRINTF("Button C  is pressed \r\n");}
	if(KeyValue[3]& 0x08){PRINTF("Button D  is pressed \r\n");}
	SMARTDMA_AccessShareRAM(0);
 }
/*!
 * @brief Main function
 */
int main(void)
{
    char ch;

    /* Init board hardware. */
    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
#if !defined(DONT_ENABLE_FLASH_PREFETCH)
    /* enable flash prefetch for better performance */
    SYSCON->FMCCR |= SYSCON_FMCCR_PREFEN_MASK;
#endif

    PRINTF("hello world.\r\n");

	SMARTDMA_InitWithoutFirmware();
	SMARTDMA_InstallFirmware(SMARTDMA_KEYSCAN_MEM_ADDR,s_smartdmaKeyscanFirmware,
								SMARTDMA_KEYSCAN_FIRMWARE_SIZE);
	SMARTDMA_InstallCallback(SmartDMA_keyscan_callback, NULL);
	NVIC_EnableIRQ(Reserved46_IRQn);
	NVIC_SetPriority(Reserved46_IRQn, 3);

	smartdmaParam.smartdma_stack 	 = (uint32_t*)g_samrtdma_stack;
	smartdmaParam.p_gpio_reg  		 = (uint32_t*)g_keyscan_gpio_register;
	smartdmaParam.p_keyvalue  		 = (uint32_t*)KeyValue;
	smartdmaParam.p_keycan_interval  = (uint32_t*)&g_keyscan_interval;
	SMARTDMA_Boot(kSMARTDMA_Keyscan_4x4, &smartdmaParam, 0x2);
    while (1)
    {
        ch = GETCHAR();
        PUTCHAR(ch);
    }
}
