#include "user_tasks.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "hardware.h"
#include "task.h"
#include "ch32x035_conf.h"

/* Local Constants */
#define UI_TASK_PRIO     5
#define UI_STK_SIZE      256

/* Local Variables */
TaskHandle_t UserInterfaceTask_Handler;

/* Local Function Prototypes */
void xProcessUITask(void *pvParameters);

/* Global Functions */
void xCreateUITask()
{
    xTaskCreate((TaskFunction_t )xProcessUITask,
                    (const char*    )"UiTsk",
                    (uint16_t       )UI_STK_SIZE,
                    (void*          )NULL,
                    (UBaseType_t    )UI_TASK_PRIO,
                    (TaskHandle_t*  )&UserInterfaceTask_Handler);
}

/* Local Functions */
void xProcessUITask(void *pvParameters)
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
	    }
}
