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
#include "hardware.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* Global define */
#define TASK1_TASK_PRIO     5
#define TASK1_STK_SIZE      256
#define TASK2_TASK_PRIO     5
#define TASK2_STK_SIZE      256

/* Global Variable */
TaskHandle_t Task1Task_Handler;
TaskHandle_t Task2Task_Handler;

/*********************************************************************
 * @fn      task1_task
 *
 * @brief   task1 program.
 *
 * @param  *pvParameters - Parameters point of task1
 *
 * @return  none
 */
void task1_task(void *pvParameters)
{
    while(1)
    {
    	if(HW_GetButtonState(0))
    	{
    		HW_SetIndicatorColor(Red);
    	}
    	else if (HW_GetButtonState(1))
    	{
    		HW_SetIndicatorColor(Green);
    	}
    	else
    	{
    		HW_SetIndicatorColor(Blue);
    	}
    }
}

/*********************************************************************
 * @fn      task2_task
 *
 * @brief   task2 program.
 *
 * @param  *pvParameters - Parameters point of task2
 *
 * @return  none
 */
void task2_task(void *pvParameters)
{
    while(1)
    {
//    	uint16_t busVoltage = HW_GetBusVoltage();
//    	uint16_t ambTemp = HW_GetAmbientTemperature();
//    	uint16_t tipCurrent = HW_GetTipCurrent();
//    	uint16_t tipVoltage = HW_GetTipTemperatureMicrovolt();
//    	uint16_t tipTemp = HW_GetTipTemperature() + ambTemp;
//    	HW_SetHeaterOutput(10000);
//
//    	if(tipTemp > 50){
//    		HW_DisableHeaterOutput();
//    	}
//    	else
//    	{
//    		HW_EnableHeaterOutput();
//    	}

    	static char msg[256];
    	sprintf(msg, "Bus: %d mV, Amb: %d °C, Tip: %d °C, Volt: %d uV, Curr: %d mA\r\n",busVoltage, ambTemp, tipTemp, tipVoltage, tipCurrent);
    	HW_SerialTransmit((uint8_t*)msg,strlen(msg));
    	HW_Delay(100);
    }
}

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

	xTaskCreate((TaskFunction_t )task2_task,
                        (const char*    )"task2",
                        (uint16_t       )TASK2_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )TASK2_TASK_PRIO,
                        (TaskHandle_t*  )&Task2Task_Handler);

    xTaskCreate((TaskFunction_t )task1_task,
                    (const char*    )"task1",
                    (uint16_t       )TASK1_STK_SIZE,
                    (void*          )NULL,
                    (UBaseType_t    )TASK1_TASK_PRIO,
                    (TaskHandle_t*  )&Task1Task_Handler);
    vTaskStartScheduler();

    while(1);

//    while(1)
//    {
//    	int16_t x = HW_GetMotionX();
//    	int16_t y = HW_GetMotionY();
//    	int16_t z = HW_GetMotionZ();
//
//		static char msg[256];
//		sprintf(msg, "%d\t%d\t%d\r\n",x,y,z);
//		HW_SerialTransmit((uint8_t*)msg,strlen(msg));
//    }


    while(1)
    {
    	if(HW_GetButtonState(0))
    	{
    		HW_SetIndicatorColor(Red);
    	}
    	else if (HW_GetButtonState(1))
    	{
    		HW_SetIndicatorColor(Green);
    	}
    	else
    	{
    		HW_SetIndicatorColor(Blue);
    	}
    	uint16_t busVoltage = HW_GetBusVoltage();
    	uint16_t ambTemp = HW_GetAmbientTemperature();
    	uint16_t tipCurrent = HW_GetTipCurrent();
    	uint16_t tipVoltage = HW_GetTipTemperatureMicrovolt();
    	uint16_t tipTemp = HW_GetTipTemperature() + ambTemp;

    	static char msg[256];
    	sprintf(msg, "Bus: %d mV, Amb: %d ï¿½C, Tip: %d ï¿½C, Volt: %d uV, Curr: %d mA\r\n",busVoltage, ambTemp, tipTemp, tipVoltage, tipCurrent);
    	HW_SerialTransmit((uint8_t*)msg,strlen(msg));
    	HW_Delay(500);
    }


}
