#include "../inc/hardware.h"
#include "../inc/pins.h"
#include "tip_table.h"

#include "FreeRTOS.h"
#include "task.h"



// Private constants ==========================================================
#define SYSTICK_DIV_TO_MILLISECOND  (SystemCoreClock / 1000)

#define HEATER_PWM_TIM_CLK          SystemCoreClock / 4 // 6 MHz
#define HEATER_PWM_TIM_PRESCALER    HEATER_PWM_TIM_CLK / HEATER_PWM_FREQUENCY - 1


// Private Types ==============================================================
typedef enum LedColor_e
{
    LedColor_Red,
    LedColor_Green,
    LedColor_Blue
} LedColor_t;

// Private interrupt service routines =========================================
//void SYS_TIME_IRQ_HANDLER(void) __attribute__((interrupt("WCH-Interrupt-fast")));

// Private function prototypes ================================================
//void _SYS_TIME_Init(void);

void _UART_Init(void);

void _USB_Init(void);

void _SPI_Init(void);
void _SPI_TransmitReceive(uint8_t *txData, uint8_t *rxData, uint16_t length);

void _ADC_Init(void);
uint16_t _ADC_ReadChannelMillivolt(uint8_t channel);
uint16_t _ADC_ReadChannelMicrovolt(uint8_t channel);
uint16_t _ADC_ReadChannel(uint8_t channel);

void Indicator_Init(void);
void Button_Init(void);
void Heater_Init(void);

void ACC_Init(void);
void ACC_SpiSelect(void);
void ACC_SpiRelease(void);
uint8_t ACC_ReadRegister(uint8_t reg);
void ACC_WriteRegister(uint8_t reg, uint8_t value);
uint8_t ACC_CheckInterrupt1(void);
uint8_t ACC_CheckInterrupt2(void);
int16_t ACC_GetX(void);
int16_t ACC_GetY(void);
int16_t ACC_GetZ(void);
uint8_t ACC_GetStatus(void);

// Private variables ==========================================================
//static uint32_t _systemTime = 0;



// Public functions ==========================================================
void HW_Init(void)
{   
    // Initialize the low-level hardware
    //_SYS_TIME_Init();
    _UART_Init();
    _USB_Init();
    _SPI_Init();
    _ADC_Init();

    // Initialize the high-level hardware
    Indicator_Init();
    HW_SetIndicatorColor(Off);

    Button_Init();

    Heater_Init();
    HW_SetHeaterOutput(0);
    HW_DisableHeaterOutput();

    ACC_Init();
}

/**
 * @brief Read the tip temperature in microvolts.
 * 
 * @return uint32_t The tip temperature in microvolts.
 */
uint32_t HW_GetTipTemperatureMicrovolt(void)
{
    uint64_t adcValueIntegrated = 0;
    for( uint8_t i = 0; i < 16; i++)
    {
        // Read the ADC value
        adcValueIntegrated += _ADC_ReadChannel(CHANNEL_T_TIP);
    }

    uint32_t uV = adcValueIntegrated  * ADC_REF_VOLTAGE_MICROVOLT / TIP_FRONTEND_GAIN / ADC_QUANTISATION_LEVELS;
    uV >>= 4;
    return uV;
}

/**
 * @brief Read the tip temperature difference to ambient in K .
 * 
 * @return int16_t The tip temperature °C.
 */
int16_t HW_GetTipTemperature(void)
{
    uint32_t uV = HW_GetTipTemperatureMicrovolt();
    return (int16_t) TIP_InterpolateTemperature(uV);
}

/**
 * @brief Read the ambient temperature channel in millivolts.
 * 
 * @return uint32_t The ambient temperature in millivolts.
 */
uint32_t HW_GetAmbientTemperatureMillivolt(void)
{
    return _ADC_ReadChannelMillivolt(CHANNEL_T_AMB);
}

/**
 * @brief Read the ambient temperature in °C.
 * 
 * @return int16_t The ambient temperature in °C.
 */
int16_t HW_GetAmbientTemperature(void)
{
    // Round to next °C value
    // Multiply by 10 to avoid floating point arithmetic
    // Add 5 to round to the nearest 10 and divide by 10 to get the value in °C
    uint16_t mV = _ADC_ReadChannelMillivolt(CHANNEL_T_AMB);
    int16_t temp = (10 * (mV - MCP_MILLIVOLT_V0C)) / MCP_TEMPERATURE_COEFFICIENT; // in .1 °C
    return (temp + 5) / 10; // in °C 
}

/**
 * @brief Read the tip current in milliamps.
 * 
 * @return uint16_t The tip current in milliamps.
 */
uint16_t HW_GetTipCurrent(void)
{   
    // Convert mV of SI8540 output to current in mA
    // I = V * RG / (RSENSE * ROUT)
    // I(mA) = V(mV) * 1000 * RG(Ohm) / (RSENSE(mOhm) * ROUT(Ohm))
    uint32_t mV = _ADC_ReadChannelMillivolt(CHANNEL_I_TIP);
	return 1000 * mV * SI8540_RG_OHM / (SI8540_RSENSE_MILLIOHM * SI8540_ROUT_OHM);
}

/**
 * @brief Read the bus voltage in millivolts.
 * 
 * @return uint16_t The bus voltage in millivolts.
 */
uint16_t HW_GetBusVoltage(void)
{   
    // Read the voltage from the ADC and scale it to the actual voltage
    // by multiplying with the voltage divider factor which is 6.666...
    uint16_t mV = _ADC_ReadChannelMillivolt(CHANNEL_VBUS_SENSE);
    return (mV * 6666) / 1000;
}

void HW_RequestBusVoltage(uint16_t millivolt)
{}

uint16_t HW_GetMaxBusCurrent()
{
	return 0;
}

/**
 * @brief Get the state of a button.
 * 
 * @param nbr The button number.
 * 
 * @return uint8_t The state of the button.
*/
uint8_t HW_GetButtonState(uint8_t nbr)
{
    switch (nbr)
    {
        case 0:
            return GPIO_ReadInputDataBit(PORT_BTN0, PIN_BTN0) == Bit_RESET;
        case 1:
            return GPIO_ReadInputDataBit(PORT_BTN1, PIN_BTN1) == Bit_RESET;
        default:
            return 0;
    }
}

/**
 * @brief Enable the heater output. The heater output value is not changed in this function.
*/
void HW_EnableHeaterOutput()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = PIN_PWM_TIP;
    GPIO_Init(PORT_PWM_TIP, &GPIO_InitStructure);
}

/**
 * @brief Disable the heater output. The heater output value is not changed in this function.
*/
void HW_DisableHeaterOutput()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = PIN_PWM_TIP;
    GPIO_Init(PORT_PWM_TIP, &GPIO_InitStructure);
    GPIO_WriteBit(PORT_PWM_TIP, PIN_PWM_TIP, Bit_RESET);
}

/**
 * @brief Set the heater output. The enable state is not changed in this function.
 * 
 * @param output The output value to set the heater to.
*/
void HW_SetHeaterOutput(uint16_t output)
{
    output = (output > HEATER_PWM_MAX) ? HEATER_PWM_MAX : output;
    // TODO: Change this to a compile-time check
    switch (CHANNEL_PWM_TIP)
    {
        case TIM_Channel_1:
            TIM_SetCompare1(TIMER_PWM_TIP, output);
            break;
        case TIM_Channel_2:
            TIM_SetCompare2(TIMER_PWM_TIP, output);
            break;
        case TIM_Channel_3:
            TIM_SetCompare3(TIMER_PWM_TIP, output);
            break;
        case TIM_Channel_4:
            TIM_SetCompare4(TIMER_PWM_TIP, output);
            break;
    }
}

/**
 * @brief Set the indicator color.
 * 
 * @param hex The hex color to set the indicator to.
*/
void HW_SetIndicatorColorHEX(uint32_t hex)
{
    uint8_t r = (hex >> 16) & 0xFF;
    uint8_t g = (hex >> 8) & 0xFF;
    uint8_t b = hex & 0xFF;
    
    // TODO: Add PMW support for full RGB colors
    GPIO_WriteBit(PORT_LED_RED, PIN_LED_RED, (r > 0) ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(PORT_LED_GREEN, PIN_LED_GREEN, (g > 0) ? Bit_RESET : Bit_SET);
    GPIO_WriteBit(PORT_LED_BLUE, PIN_LED_BLUE, (b > 0) ? Bit_RESET : Bit_SET);
}

/**
 * @brief Set the indicator color.
 * 
 * @param color The color to set the indicator to.
*/
void HW_SetIndicatorColor(IndicatorColor_t color)
{
    HW_SetIndicatorColorHEX((uint32_t)color);
}

/**
 * @brief Get the shake detected state.
 * 
 * @return uint8_t The shake detected state. 1 if shake detected, 0 otherwise.
*/
uint8_t HW_GetMotionState(void)
{
    // Any motion is configured to generate an interrupt on INT1
    return ACC_CheckInterrupt1();
}

/**
 * @brief Get the motion X-axis value.
 * 
 * @return int16_t The X-axis value of the motion sensor.
*/
int16_t HW_GetMotionX(void)
{
    return ACC_GetX();
}

/**
 * @brief Get the motion Y-axis value.
 * 
 * @return int16_t The Y-axis value of the motion sensor.
*/
int16_t HW_GetMotionY(void)
{
    return ACC_GetY();
}

/**
 * @brief Get the motion Z-axis value.
 * 
 * @return int16_t The Z-axis value of the motion sensor.
*/
int16_t HW_GetMotionZ(void)
{
    return ACC_GetZ();
}

// /**
//  * @brief Get the system timestamp in milliseconds.
//  * @note The system timestamp is the time since the system started in milliseconds. 
//  *       The timestamp is a 32-bit value and will overflow after 49.7 days.
//  * 
//  * @return uint32_t The system timestamp in milliseconds.
// */
// uint32_t HW_GetSystemTimestamp(void)
// {
//     return HW_GetSystemTimstampMicrosecond() / 1000;
// }

// /**
//  * @brief Get the system ticks.
//  * 
//  * @return uint64_t The system ticks.
// */
// uint64_t HW_GetSystemTimstampMicrosecond(void)
// {
    
//     return  ((uint64_t) _systemTime << 16) | TIMER_SYS_TIME->CNT;
// }

/**
 * @brief Delay the system for a certain amount of time.
 * 
 * @param milliseconds The time to delay in milliseconds.
*/
void HW_Delay(uint32_t milliseconds)
{
    vTaskDelay(milliseconds/portTICK_PERIOD_MS);
}

/**
 * @brief Transmit data to the USB port.
 * 
 * @param data The data buffer to transmit.
 * @param length The length of the data to transmit.
*/
void HW_USBTransmit(uint8_t *data, uint16_t length)
{}

/**
 * @brief   Receive data from the USB port.
 * 
 * @param data The data buffer to store the received data.
 * @param length The length of the data to receive. 
 */
void HW_USBReceive(uint8_t *data, uint16_t length)
{}

/**
 * @brief Transmit data to the serial port.
 * 
 * @param data The data buffer to transmit.
 * @param length The length of the data to transmit.
*/
void HW_SerialTransmit(uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        while (USART_GetFlagStatus(UART, USART_FLAG_TXE) == RESET);
        USART_SendData(UART, data[i]);
    }
}

/**
 * @brief Receive data from the serial port.
 * 
 * @param data The data buffer to store the received data.
 * @param length The length of the data to receive.
*/
void HW_SerialReceive(uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        while (USART_GetFlagStatus(UART, USART_FLAG_RXNE) == RESET);
        data[i] = USART_ReceiveData(UART);
    }
}

// Private functions ==============================================================================================


// Low-level hardware functions ===================================================================================
// /**
//  * @brief Initialize the system time. 
// */
// void _SYS_TIME_Init(void)
// {

//     RCC_APB1PeriphClockCmd(PERIPH_TIM_SYS_TIME, ENABLE);

//     // Timer Base Initialization
// 	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};
// 	TIM_TimeBaseInitStructure.TIM_Period = 65535; // 16-bit counter equivalent to 65.535 ms
// 	TIM_TimeBaseInitStructure.TIM_Prescaler = (SystemCoreClock / 1000000 ) - 1 ; // 1 us per tick
// 	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
// 	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
// 	TIM_TimeBaseInit( TIMER_SYS_TIME, &TIM_TimeBaseInitStructure);

//     TIM_ITConfig(TIMER_SYS_TIME, TIM_IT_Update, ENABLE);
    
//     NVIC_InitTypeDef NVIC_InitStruct;
//     NVIC_InitStruct.NVIC_IRQChannel = SYS_TIME_IRQ;
//     NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 5;
//     NVIC_InitStruct.NVIC_IRQChannelSubPriority = 5;
//     NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&NVIC_InitStruct);
	    
//     TIM_Cmd(TIMER_SYS_TIME, ENABLE);
// }

// void SYS_TIME_IRQ_HANDLER(void)
// {
//     if (TIM_GetITStatus(TIMER_SYS_TIME, TIM_IT_Update) == SET)
//     {
//         _systemTime++;
//         TIM_ClearITPendingBit(TIMER_SYS_TIME, TIM_IT_Update); // 
//     }
// }

/**
 * @brief Initialize the serial port. The serial port configuration is defined in the hardware.h file.
*/
void _UART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(PERIPH_UART, ENABLE);
    RCC_APB2PeriphClockCmd(PERIPH_UART_RX | PERIPH_UART_TX, ENABLE);

    GPIO_InitStructure.GPIO_Pin = PIN_UART_TX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(PORT_UART_TX, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = PIN_UART_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(PORT_UART_RX, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = UART_BAUDRATE;
    USART_InitStructure.USART_WordLength = UART_WORDLENGTH;
    USART_InitStructure.USART_StopBits = UART_STOPBITS;
    USART_InitStructure.USART_Parity = UART_PARITY;
    USART_InitStructure.USART_HardwareFlowControl = UART_FLOW_CONTROL;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(UART, &USART_InitStructure);
    USART_Cmd(UART, ENABLE);
}

/**
 * @brief Initialize USB periphery for CDC communication.
*/
void _USB_Init(void)
{
  
}

/**
 * @brief Initialize the SPI.
*/
void _SPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(PERIPH_SPI_MISO | PERIPH_SPI_SCK | PERIPH_SPI_MOSI, ENABLE);
   
    GPIO_InitStructure.GPIO_Pin = PIN_SPI_SCK;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_SPI_SCK, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_SPI_MISO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(PORT_SPI_MISO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_SPI_MOSI;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(PORT_SPI_MOSI, &GPIO_InitStructure);

    GPIO_SetBits(PORT_SPI_CS, PIN_SPI_CS);
    
    SPI_InitTypeDef  SPI_InitStructure = {0};
    RCC_APB2PeriphClockCmd(PERIPH_SPI, ENABLE);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI, &SPI_InitStructure);

    SPI_Cmd(SPI, ENABLE);
}

/**
 * @brief Transmit and receive data over the SPI bus.
 * 
 * @param txData The data to transmit.
 * @param rxData The data buffer to store the received data.
 * @param length The length of the data to transmit and receive.
*/
void _SPI_TransmitReceive(uint8_t *txData, uint8_t *rxData, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(SPI, txData[i]);
        while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
        rxData[i] = SPI_I2S_ReceiveData(SPI);
    }
}

/**
* @brief Initialize the analog inputs.
 */
void _ADC_Init(void)
{
    // Enable the clock
    RCC_APB2PeriphClockCmd(PERIPH_T_TIP | PERIPH_T_AMB | PERIPH_I_TIP, ENABLE);
    RCC_APB2PeriphClockCmd(PERIPH_ADC, ENABLE);

    // GPIO Initialization
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

    GPIO_InitStructure.GPIO_Pin = PIN_T_TIP;
    GPIO_Init(PORT_T_TIP, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_T_AMB;
    GPIO_Init(PORT_T_AMB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_I_TIP;
    GPIO_Init(PORT_I_TIP, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_VBUS_SENSE;
    GPIO_Init(PORT_VBUS_SENSE, &GPIO_InitStructure);

    // ADC Initialization
    ADC_InitTypeDef ADC_InitStructure = {0};
    
    ADC_DeInit(ADC);
    ADC_CLKConfig(ADC, ADC_CLK_Div6);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC, &ADC_InitStructure);
    ADC_Cmd(ADC, ENABLE);

}

/**
 * @brief Read the value of an analog input channel.
 * 
 * @param channel The channel to read.
 * @return uint16_t The value of the channel in millivolts.
 */
uint16_t _ADC_ReadChannelMillivolt(uint8_t channel)
{
    uint16_t val;
    ADC_RegularChannelConfig(ADC, channel, 1, ADC_SampleTime_11Cycles);
    ADC_SoftwareStartConvCmd(ADC, ENABLE);

    while(!ADC_GetFlagStatus(ADC, ADC_FLAG_EOC));
    val = ADC_GetConversionValue(ADC);
    return ( (uint32_t) val * ADC_REF_VOLTAGE_MILLIVOLT) / ADC_QUANTISATION_LEVELS;
}

/**
 * @brief Read the value of an analog input channel.
 * 
 * @param channel The channel to read.
 * @return uint16_t The value of the channel in millivolts.
 */
uint16_t _ADC_ReadChannelMicrovolt(uint8_t channel)
{
    uint16_t val;
    ADC_RegularChannelConfig(ADC, channel, 1, ADC_SampleTime_11Cycles);
    ADC_SoftwareStartConvCmd(ADC, ENABLE);

    while(!ADC_GetFlagStatus(ADC, ADC_FLAG_EOC));
    val = ADC_GetConversionValue(ADC);
    return ( (uint32_t) val * ADC_REF_VOLTAGE_MICROVOLT) / ADC_QUANTISATION_LEVELS;
}

/**
 * @brief Read the value of an analog input channel.
 * 
 * @param channel The channel to read.
 * @return uint16_t The value of the channel.
 */
uint16_t _ADC_ReadChannel(uint8_t channel)
{
    uint16_t val;
    ADC_RegularChannelConfig(ADC, channel, 1, ADC_SampleTime_11Cycles);
    ADC_SoftwareStartConvCmd(ADC, ENABLE);

    while(!ADC_GetFlagStatus(ADC, ADC_FLAG_EOC));
    val = ADC_GetConversionValue(ADC);
    return val;
}
			

// High-level hardware functions =================================================================================

/**
 * @brief Initialize the accelerometer.
*/
void ACC_Init(void)
{
    // GPIO Initialization
    // SPI is already initialized

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(PERIPH_SPI_CS | PERIPH_ACC_INT1 | PERIPH_ACC_INT2 , ENABLE);

    GPIO_InitStructure.GPIO_Pin = PIN_SPI_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_SPI_CS, &GPIO_InitStructure);

    GPIO_WriteBit(PORT_SPI_CS, PIN_SPI_CS, Bit_SET);

    GPIO_InitStructure.GPIO_Pin = PIN_ACC_INT1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(PORT_ACC_INT1, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_ACC_INT2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(PORT_ACC_INT2, &GPIO_InitStructure);


    // Initialize the accelerometer (to be replaced by a proper driver)
    // Type: MEMSIC MC3479
    // General configuration: ANYMOTION detection is enabled and generates an interrupt on INT1
    //                        ANYMOTION threshold is set to an appropriate value

    // TODO the implementation of the accelerometer initialization
    // ACC_WriteRegister(0x06, (1<<2)); // Any motion interrupt enable
    ACC_WriteRegister(0x07, (1<<0)); // State WAKE
    ACC_WriteRegister(0x08, 0x08 | (3 << 0)); // 200 Hz sampling rate
    // ACC_WriteRegister(0x09, (1<<2)); // Any motion enable
    ACC_WriteRegister(0x20, 0x00 | (1<< 3) | (1<<0)); // +/- 2g range, low pass filter enabled (fc = IDR/4.255)
    // ACC_WriteRegister(0x43, 0x0A); // Any motion threshold
    // ACC_WriteRegister(0x44, 0x0A); // Any debounce duration

}

/**
 * @brief Get the X-axis value of the accelerometer.
 * 
 * @return uint16_t The X-axis value of the accelerometer.
*/
int16_t ACC_GetX(void)
{
    uint8_t LSB = ACC_ReadRegister(0x0D);
    uint8_t MSB = ACC_ReadRegister(0x0E);
    return (int16_t)((((uint16_t) MSB) << 8) | LSB);
}

/**
 * @brief Get the Y-axis value of the accelerometer.
 * 
 * @return uint16_t The Y-axis value of the accelerometer.
*/
int16_t ACC_GetY(void)
{
    uint8_t LSB = ACC_ReadRegister(0x0F);
    uint8_t MSB = ACC_ReadRegister(0x10);
    return (int16_t)((((uint16_t) MSB) << 8) | LSB);
}

/**
 * @brief Get the Z-axis value of the accelerometer.
 * 
 * @return uint16_t The Z-axis value of the accelerometer.
*/
int16_t ACC_GetZ(void)
{
    uint8_t LSB = ACC_ReadRegister(0x11);
    uint8_t MSB = ACC_ReadRegister(0x12);
    return (int16_t)((((uint16_t) MSB) << 8) | LSB);
}

/**
 * @brief Get the status of the accelerometer.
 * 
 * @return uint8_t The status of the accelerometer.
*/
uint8_t ACC_GetStatus(void)
{
    return ACC_ReadRegister(0x13);
}

/**
 * @brief Check if the accelerometer interrupt 1 is active.
 * 
 * @return uint8_t The state of the interrupt. 1 if the interrupt is active, 0 otherwise.
*/
uint8_t ACC_CheckInterrupt1(void)
{
    uint8_t value = GPIO_ReadInputDataBit(PORT_ACC_INT1, PIN_ACC_INT1) == Bit_RESET;
    ACC_WriteRegister(0x14,0x00); // Clear the interrupt
    
    return value;
}

/**
 * @brief Check if the accelerometer interrupt 2 is active.
 * 
 * @return uint8_t The state of the interrupt. 1 if the interrupt is active, 0 otherwise.
*/
uint8_t ACC_CheckInterrupt2(void)
{
    uint8_t value =  GPIO_ReadInputDataBit(PORT_ACC_INT2, PIN_ACC_INT2) == Bit_RESET;
    ACC_WriteRegister(0x14,0x00); // Clear the interrupt

    return value;
}

/**
 * @brief Select the SPI bus for the accelerometer.
*/
void ACC_SpiSelect(void)
{
    GPIO_ResetBits(PORT_SPI_CS, PIN_SPI_CS);
}

/**
 * @brief Release the SPI bus from the accelerometer.
*/
void ACC_SpiRelease(void)
{
    GPIO_SetBits(PORT_SPI_CS, PIN_SPI_CS);
}

/**
 * @brief Read a value from an accelerometer register.
 * 
 * @param reg The register to read from.
 * @return uint8_t The value of the register.
*/
uint8_t ACC_ReadRegister(uint8_t reg)
{
    uint8_t txBuffer[3] = {
        reg | 0x80,
        0x00,
        0x00
    };

    uint8_t rxBuffer[3] = {0};

    ACC_SpiSelect();
    _SPI_TransmitReceive(txBuffer, rxBuffer, 3);
    ACC_SpiRelease();

    return rxBuffer[2];
}

/**
 * @brief Write a value to an accelerometer register.
 * 
 * @param reg The register to write to.
 * @param value The value to write to the register.
*/
void ACC_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t txBuffer[2] = {
        reg & 0x7F,
        value
    };

    ACC_SpiSelect();
    _SPI_TransmitReceive(txBuffer, NULL, 2);
    ACC_SpiRelease();
}

/**
 * @brief Initialize the buttons.
*/
void Button_Init(void)
{
    RCC_APB2PeriphClockCmd(PERIPH_BTN0 | PERIPH_BTN1, ENABLE);
    
    // GPIO Initialization
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = PIN_BTN0;
    GPIO_Init(PORT_BTN0, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_BTN1;
    GPIO_Init(PORT_BTN1, &GPIO_InitStructure);

}

/**
 * @brief Initialize the indicator.
*/
void Indicator_Init(void)
{
    RCC_APB2PeriphClockCmd(PERIPH_LED_RED | PERIPH_LED_GREEN | PERIPH_LED_BLUE, ENABLE);
    
    // GPIO Initialization
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = PIN_LED_RED;
    GPIO_Init(PORT_LED_RED, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_LED_GREEN;
    GPIO_Init(PORT_LED_GREEN, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_LED_BLUE;
    GPIO_Init(PORT_LED_BLUE, &GPIO_InitStructure);

    // TODO: Add PMW support for full RGB colors
}

/**
 * @brief Initialize the heater output.
*/
void Heater_Init(void)
{
    // Enable the clock 
    RCC_APB2PeriphClockCmd(PERIPH_PWM_TIP, ENABLE);
    RCC_APB1PeriphClockCmd(PERIPH_TIM_PWM_TIP, ENABLE);

    // GPIO Initialization
	GPIO_InitTypeDef GPIO_InitStructure={0};
	GPIO_InitStructure.GPIO_Pin = PIN_PWM_TIP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( PORT_PWM_TIP, &GPIO_InitStructure );

    

    // Timer Base Initialization
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};
	TIM_TimeBaseInitStructure.TIM_Period = 65535; // 16-bit
	TIM_TimeBaseInitStructure.TIM_Prescaler = HEATER_PWM_TIM_PRESCALER;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV4; 
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIMER_PWM_TIP, &TIM_TimeBaseInitStructure);
	
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // Edge-aligned PWM mode 1

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 10000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
    // TODO: Change this to a compile-time check
    switch (CHANNEL_PWM_TIP)
    {
        case TIM_Channel_1:
            TIM_OC1Init( TIMER_PWM_TIP, &TIM_OCInitStructure );
            break;
        case TIM_Channel_2:
            TIM_OC2Init( TIMER_PWM_TIP, &TIM_OCInitStructure );
            break;
        case TIM_Channel_3:
            TIM_OC3Init( TIMER_PWM_TIP, &TIM_OCInitStructure );
            break;
        case TIM_Channel_4:
            TIM_OC4Init( TIMER_PWM_TIP, &TIM_OCInitStructure );
            break;
    }
    
	TIM_CtrlPWMOutputs(TIMER_PWM_TIP, ENABLE );
	TIM_OC1PreloadConfig( TIMER_PWM_TIP, TIM_OCPreload_Disable );
	TIM_ARRPreloadConfig( TIMER_PWM_TIP, ENABLE );
	TIM_Cmd( TIMER_PWM_TIP, ENABLE );

//	TIM_SetCompare4(TIMER_PWM_TIP, 50000);
}
