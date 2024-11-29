#include "user_tasks.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "hardware.h"
#include "task.h"
#include "ch32x035_conf.h"

/* Local Constants */
#define USB_PD_TASK_PRIO     5
#define USB_PD_STK_SIZE      256

/* Local Variables */
TaskHandle_t UsbPdTask_Handler;

/* Local Function Prototypes */
void xProcessUsbPdTask(void *pvParameters);

/* Global Functions */
void xCreateUsbPdTask()
{
    xTaskCreate((TaskFunction_t )xProcessUsbPdTask,
                        (const char*    )"UsbPdTsk",
                        (uint16_t       )USB_PD_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )USB_PD_TASK_PRIO,
                        (TaskHandle_t*  )&UsbPdTask_Handler);
}

/* Local Functions */
void xProcessUsbPdTask(void *pvParameters)
{
	// Handle USB PD connection etc
    while(1)
    {
        vTaskDelay(1000);
    }
}
