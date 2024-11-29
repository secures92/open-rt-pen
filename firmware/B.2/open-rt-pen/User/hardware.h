#ifndef USER_HARDWARE_H_
#define USER_HARDWARE_H_

#include <stdint.h>


// Public constants ===========================================================
#define HEATER_PWM_FREQUENCY        ((uint32_t) 20000 )
#define HEATER_PWM_PERIOD			((uint32_t) 100)
#define HEATER_PWM_MAX              ((uint16_t)(0.95f * HEATER_PWM_PERIOD))

#define UART_BAUDRATE               115200
#define UART_WORDLENGTH             USART_WordLength_8b
#define UART_PARITY                 USART_Parity_No
#define UART_STOPBITS               USART_StopBits_1
#define UART_FLOW_CONTROL           USART_HardwareFlowControl_None

#define ADC_REF_VOLTAGE_MILLIVOLT   3300
#define ADC_REF_VOLTAGE_MICROVOLT	ADC_REF_VOLTAGE_MILLIVOLT * 1000
#define ADC_BITS                    12
#define ADC_QUANTISATION_LEVELS		(1 << ADC_BITS)

#define MCP_MILLIVOLT_V0C			500	// Millivolt at 0�C
#define MCP_TEMPERATURE_COEFFICIENT	10  // Temperature Coefficient 10 mV/�C

#define SI8540_RSENSE_MILLIOHM		10
#define SI8540_RG_OHM				33
#define SI8540_ROUT_OHM				2200

#define TIP_FRONTEND_GAIN	        304

// Public types ===============================================================
typedef enum IndicatorColor_e
{
    Blue        = 0x000000FF,
    Green       = 0x0000FF00,
    Red         = 0x00FF0000,
    Yellow      = 0x00FFFF00,
    Cyan        = 0x0000FFFF,
    Magenta     = 0x00FF00FF,
    White       = 0x00FFFFFF,
    Off         = 0x00000000
} IndicatorColor_t;


// Public function prototypes =================================================

void HW_Init(void);

int16_t HW_GetTipTemperature(void);
uint32_t HW_GetTipTemperatureMicrovolt(void);
int16_t HW_GetAmbientTemperature(void);
uint32_t HW_GetAmbientTemperatureMillivolt(void);
uint16_t HW_GetTipCurrent(void);

uint16_t HW_GetBusVoltage(void);
void HW_RequestBusVoltage(uint16_t millivolt);
uint16_t HW_GetMaxBusCurrent(void);

uint8_t HW_GetButtonState(uint8_t nbr);

void HW_EnableHeaterOutput(void);
void HW_DisableHeaterOutput(void);
void HW_SetHeaterOutput(uint16_t value);

void HW_SetIndicatorColorHEX(uint32_t hex);
void HW_SetIndicatorColor(IndicatorColor_t color);

// uint8_t HW_GetMotionState(void);
int16_t HW_GetMotionX(void);
int16_t HW_GetMotionY(void);
int16_t HW_GetMotionZ(void);

uint32_t HW_GetSystemTimestamp(void);
uint64_t HW_GetSystemTicks(void);
void HW_Delay(uint32_t milliseconds);

void HW_USBTransmit(uint8_t *data, uint16_t length);
void HW_USBReceive(uint8_t *data, uint16_t length);

void HW_SerialTransmit(uint8_t *data, uint16_t length);
void HW_SerialReceive(uint8_t *data, uint16_t length);



#endif /* USER_HARDWARE_H_ */
