/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/26
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "debug.h"
#include "FreeRTOS.h"
#include "hardware.h"
#include "task.h"
#include "ch32x035_conf.h"
#include "Tasks/user_tasks.h"


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
	HW_Init();
	HW_SetHeaterOutput(0);


	printf("SystemClk:%d\r\n",SystemCoreClock);
    printf("FreeRTOS Kernel Version:%s\r\n",tskKERNEL_VERSION_NUMBER);

	xCreateUITask();
	xCreateTempCtrlTask();
	xCreateUsbPdTask();

    vTaskStartScheduler();

    while(1)
    {
        printf("shouldn't run at here!!\n");
    }
}

