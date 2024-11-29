#include "user_tasks.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "hardware.h"
#include "task.h"
#include "ch32x035_conf.h"

/* Local Constants */
#define TEMP_CTRL_TASK_PRIO     5
#define TEMP_CTRL_STK_SIZE      256

/* Local Variables */
TaskHandle_t TemperatureControllerTask_Handler;

/* Local Function Prototypes */
void xProcessTempCtrlTask(void *pvParameters);

/* Global Functions */
void xCreateTempCtrlTask()
{
    xTaskCreate((TaskFunction_t )xProcessTempCtrlTask,
                        (const char*    )"TempCtrlTsk",
                        (uint16_t       )TEMP_CTRL_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )TEMP_CTRL_TASK_PRIO,
                        (TaskHandle_t*  )&TemperatureControllerTask_Handler);
}

/* Local Functions */
void xProcessTempCtrlTask(void *pvParameters)
{
    while(1)
    {
    	HW_SetIndicatorColor(Blue);
        vTaskDelay(1000);
        HW_SetIndicatorColor(Off);
        vTaskDelay(1000);
    }
}
