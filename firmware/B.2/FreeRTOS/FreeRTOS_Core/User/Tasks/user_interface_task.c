#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

#include "user_interface_task.h"
#include "hardware.h"

TaskHandle_t UserInterfaceTask_Handler;

void TASK_UserInterface(void *pvParameters);

void TASK_RegisterUserInterfaceTask()
{
	xTaskCreate(
			(TaskFunction_t )TASK_UserInterface,
			(const char*    )"UserInterface",
			(uint16_t       )USER_INTERFACE_TASK_STACK_SIZE,
			(void*          ) NULL,
			(UBaseType_t    )USER_INTERFACE_TASK_PRIORITY,
			(TaskHandle_t*  )&UserInterfaceTask_Handler
	);
}

void TASK_UserInterface(void *pvParameters)
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
