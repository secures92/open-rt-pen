/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

#include "stm32g0xx_ll_ucpd.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_dma.h"

#include "stm32g0xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int32_t HW_GetChipTemperature(void);
int32_t HW_GetTipTemperature(void);
uint32_t HW_GetTipCurrent(void);
uint32_t HW_GetReferenceVoltage(void);
uint32_t HW_GetSupplyVoltage(void);
uint32_t HW_GetBusVoltage(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC1_NO_CHANNELS 6
#define TIP_VOLTAGE_Pin GPIO_PIN_1
#define TIP_VOLTAGE_GPIO_Port GPIOA
#define HEATER_CURRENT_Pin GPIO_PIN_5
#define HEATER_CURRENT_GPIO_Port GPIOA
#define VBUS_SENSE_Pin GPIO_PIN_1
#define VBUS_SENSE_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_2
#define BTN1_GPIO_Port GPIOD
#define BTN0_Pin GPIO_PIN_3
#define BTN0_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_8
#define LED0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
