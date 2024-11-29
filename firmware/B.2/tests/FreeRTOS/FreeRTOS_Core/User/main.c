/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 *GPIO routine:
 *PA0 push-pull output.
 *
 ***Only PA0--PA15 and PC16--PC17 support input pull-down.
 */

#include "ch32x035.h"
#include "ch32x035_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include "hardware.h"
#include "temperature_control_task.h"
#include "user_interface_task.h"

/* Global define */


/* Global Variable */


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
    HW_Init();


//    TASK_RegisterTemperatureControlTask();
//    TASK_RegisterUserInterfaceTask();
//    vTaskStartScheduler();

    // Should not be reached
    while(1)
    {
    	int16_t x = HW_GetMotionX();
    	int16_t y = HW_GetMotionY();
    	int16_t z = HW_GetMotionZ();

		static char msg[256];
		sprintf(msg, "%d\t%d\t%d\r\n",x,y,z);
		HW_SerialTransmit((uint8_t*)msg,strlen(msg));
    }

}
