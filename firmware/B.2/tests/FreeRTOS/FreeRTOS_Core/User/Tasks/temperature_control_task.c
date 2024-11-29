#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

#include "temperature_control_task.h"
#include "hardware.h"

TaskHandle_t TemperatureControllerTask_Handler;

void TASK_TemperatureController(void *pvParameters);

void TASK_RegisterTemperatureControlTask()
{
	xTaskCreate(
			(TaskFunction_t )TASK_TemperatureController,
			(const char*    )"TemperatureController",
			(uint16_t       )TEMPERATURE_CONTROLLER_TASK_STACK_SIZE,
			(void*          ) NULL,
			(UBaseType_t    )TEMPERATURE_CONTROLLER_TASK_PRIORITY,
			(TaskHandle_t*  )&TemperatureControllerTask_Handler
	);
}

void TASK_TemperatureController(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = TEMPERATURE_CONTROLLER_TASK_TICK_PERIOD_MS / portTICK_PERIOD_MS;
	
	xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
		HW_DisableHeaterOutput();
		HW_Delay(4);

    	uint16_t busVoltage = HW_GetBusVoltage();
    	uint16_t ambTemp = HW_GetAmbientTemperature();
    	uint16_t tipCurrent = HW_GetTipCurrent();
    	uint16_t tipVoltage = HW_GetTipTemperatureMicrovolt();
    	uint16_t tipTemp = HW_GetTipTemperature() + ambTemp;

		if (tipTemp < 50)
		{
			HW_SetHeaterOutput(50000);
		}
		else
		{
			HW_SetHeaterOutput(0);
		}

    	static char msg[128];
    	sprintf(msg, "Bus: %d mV, Amb: %d °C, Tip: %d °C, Volt: %d uV, Curr: %d mA\r\n", busVoltage, ambTemp, tipTemp, tipVoltage, tipCurrent);
    	HW_SerialTransmit((uint8_t*)msg,strlen(msg));

		HW_EnableHeaterOutput(); 
    	
		
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}
