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

#include "debug.h"
#include "hardware.h"
#include "string.h"

/* Global define */

/* Global Variable */

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void) {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);
	HW_Init();
	HW_SetHeaterOutput(20);

	while (1) {

		uint16_t busVoltage = HW_GetBusVoltage();
		uint16_t ambTemp = HW_GetAmbientTemperature();
		uint16_t tipCurrent = HW_GetTipCurrent();

		HW_DisableHeaterOutput();
		Delay_Ms(5);
		uint32_t tipVoltage = HW_GetTipTemperatureMicrovolt();
		uint32_t tipTemp = HW_GetTipTemperature() + ambTemp;

		if (tipTemp > 350)
		{
			HW_DisableHeaterOutput();
		}
		else
		{
			HW_EnableHeaterOutput();
		}


		static char msg[256];
		sprintf(msg,
				"Bus: %d mV, Amb: %d C, Tip: %d C, Volt: %lu uV, Curr: %d mA\r\n",
				busVoltage, ambTemp, tipTemp, tipVoltage, tipCurrent);

		HW_SerialTransmit((uint8_t*) msg, strlen(msg));


		Delay_Ms(100);
	}
}
