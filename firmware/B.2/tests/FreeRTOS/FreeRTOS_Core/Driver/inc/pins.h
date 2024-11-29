#ifndef USER_PINS_H_
#define USER_PINS_H_

#include "ch32x035.h"


// ============================================================================
// Pin definitions according to the schematic
// ============================================================================

// SystemTime ================================================================
#define PERIPH_TIM_SYS_TIME RCC_APB1Periph_TIM3
#define TIMER_SYS_TIME      TIM3
#define SYS_TIME_IRQ        TIM3_IRQn
#define SYS_TIME_IRQ_HANDLER TIM3_IRQHandler

// LEDs =======================================================================
#define PIN_LED0            GPIO_Pin_4
#define PORT_LED0           GPIOB
#define PERIPH_LED0         RCC_APB2Periph_GPIOB

#define PIN_LED1            GPIO_Pin_7
#define PORT_LED1           GPIOB
#define PERIPH_LED1         RCC_APB2Periph_GPIOB

#define PIN_LED2            GPIO_Pin_6
#define PORT_LED2           GPIOB
#define PERIPH_LED2         RCC_APB2Periph_GPIOB

// Buttons ===================================================================
#define PIN_BTN0            GPIO_Pin_1
#define PORT_BTN0           GPIOB
#define PERIPH_BTN0         RCC_APB2Periph_GPIOB

#define PIN_BTN1            GPIO_Pin_8
#define PORT_BTN1           GPIOB
#define PERIPH_BTN1         RCC_APB2Periph_GPIOB
 
// Heater =====================================================================
#define PIN_PWM_TIP         GPIO_Pin_0
#define PORT_PWM_TIP        GPIOC
#define PERIPH_PWM_TIP      RCC_APB2Periph_GPIOC
#define TIMER_PWM_TIP       TIM2
#define CHANNEL_PWM_TIP     TIM_Channel_4
#define PERIPH_TIM_PWM_TIP  RCC_APB1Periph_TIM2

// ADC =======================================================================
#define PIN_T_TIP           GPIO_Pin_1
#define PORT_T_TIP          GPIOA
#define PERIPH_T_TIP        RCC_APB2Periph_GPIOA
#define CHANNEL_T_TIP       ADC_Channel_1

#define PIN_T_AMB           GPIO_Pin_3
#define PORT_T_AMB          GPIOA
#define PERIPH_T_AMB        RCC_APB2Periph_GPIOA
#define CHANNEL_T_AMB       ADC_Channel_3

#define PIN_I_TIP           GPIO_Pin_2
#define PORT_I_TIP          GPIOA
#define PERIPH_I_TIP        RCC_APB2Periph_GPIOA
#define CHANNEL_I_TIP       ADC_Channel_2

#define PIN_VBUS_SENSE      GPIO_Pin_0
#define PORT_VBUS_SENSE     GPIOA
#define PERIPH_VBUS_SENSE   RCC_APB2Periph_GPIOA
#define CHANNEL_VBUS_SENSE  ADC_Channel_0

#define ADC                 ADC1
#define PERIPH_ADC          RCC_APB2Periph_ADC1

// USB PD =====================================================================
// #define PIN_USB_CC1         -1
// #define PORT_USB_CC1        -1
// #define PIN_USB_CC2         -1
// #define PORT_USB_CC2        -1

// Accelerometer ==============================================================
#define PIN_ACC_INT1        GPIO_Pin_0
#define PORT_ACC_INT1       GPIOB
#define PERIPH_ACC_INT1     RCC_APB2Periph_GPIOB

#define PIN_ACC_INT2        GPIO_Pin_3
#define PORT_ACC_INT2       GPIOB
#define PERIPH_ACC_INT2     RCC_APB2Periph_GPIOB

#define PIN_SPI_CS          GPIO_Pin_4
#define PORT_SPI_CS         GPIOA
#define PERIPH_SPI_CS       RCC_APB2Periph_GPIOA

#define PIN_SPI_SCK         GPIO_Pin_5
#define PORT_SPI_SCK        GPIOA
#define PERIPH_SPI_SCK      RCC_APB2Periph_GPIOA

#define PIN_SPI_MISO        GPIO_Pin_6
#define PORT_SPI_MISO       GPIOA
#define PERIPH_SPI_MISO     RCC_APB2Periph_GPIOA

#define PIN_SPI_MOSI        GPIO_Pin_7
#define PORT_SPI_MOSI       GPIOA
#define PERIPH_SPI_MOSI     RCC_APB2Periph_GPIOA

#define SPI                 SPI1
#define PERIPH_SPI          RCC_APB2Periph_SPI1

// UART =======================================================================
#define PIN_UART_TX         GPIO_Pin_10
#define PORT_UART_TX        GPIOB
#define PERIPH_UART_TX      RCC_APB2Periph_GPIOB

#define PIN_UART_RX         GPIO_Pin_11
#define PORT_UART_RX        GPIOB
#define PERIPH_UART_RX      RCC_APB2Periph_GPIOB

#define UART                USART1
#define PERIPH_UART         RCC_APB2Periph_USART1


// ============================================================================
// Pin Renaming for better readability 
// ============================================================================

// LEDs =======================================================================
#define PIN_LED_RED         PIN_LED2
#define PORT_LED_RED        PORT_LED2
#define PERIPH_LED_RED      PERIPH_LED2

#define PIN_LED_GREEN       PIN_LED0
#define PORT_LED_GREEN      PORT_LED0
#define PERIPH_LED_GREEN    PERIPH_LED0

#define PIN_LED_BLUE        PIN_LED1
#define PORT_LED_BLUE       PORT_LED1
#define PERIPH_LED_BLUE     PERIPH_LED1


// Accelerometer ==============================================================
#define PIN_ACC_CS          PIN_SPI_CS
#define PORT_ACC_CS         PORT_SPI_CS
#define PERIPH_ACC_CS       PERIPH_SPI_CS

#endif /* USER_PINS_H_ */
