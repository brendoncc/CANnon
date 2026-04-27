/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/**
 * @defgroup Hardware_Map Hardware Resource Allocation Map
 * @brief Master ledger of hardware resources used in the CANnon project.
 *
 * =========================================================================
 * HARDWARE RESOURCE ALLOCATION MAP
 * =========================================================================
 *
 * -- TIMERS --
 * TIM2: CAN Power Clock (PWM Channel 2)
 * TIM4: Status LEDs PWM (CH2: Green, CH3: Red, CH4: Blue)
 *
 * -- CONNECTIVITY --
 * USART2: Debug Console (ST-Link VCP)
 * USART4: UART CLI Interface
 * I2C2:   HDC2021 Temperature & Humidity Sensor
 * FDCAN1: CAN Bus Interface
 * SPI2:   BlueNRG-M0L BLE Module Interface
 * USBFS: USB CDC Interface (Highest Priority CLI)
 *
 * -- GPIO --
 * GPIOB_1:  BLE_RESET (Output)
 * GPIOB_4:  CAN_PWR_EN (Output)
 * GPIOB_12: BLE_SPI_CS (Output)
 *
 * -- EXTI LINES (EXTI4_15_IRQn) --
 * EXTI_Line13: BLE Module SPI IRQ (Rising Edge - GPIOB_13)
 * EXTI_Line14: User Button (Falling Edge - GPIOB_14)
 *
 * -- DMA CHANNELS --
 * DMA1_Channel2_3: USART2 TX (Debug Console / Logging)
 *
 * -- SYSTEM --
 * IWDG:  Independent Watchdog Timer
 *
 * =========================================================================
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
#include "hci_tl_interface.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "slcan.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef enum {
    MODE_CLI,      // Human interactive mode (Default)
    MODE_SLCAN     // Machine-to-Machine bridge mode
} system_mode_t;

extern system_mode_t current_system_mode;

typedef enum
{
	CLI_PORT_UART, // Fallback priority
	CLI_PORT_BLE,  // Medium priority
	CLI_PORT_USB   // Highest priority
} cli_port_t;

extern cli_port_t current_cli_port;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void breathe_LED(void);
void cli_rx(char c);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLE_RESET_Pin GPIO_PIN_1
#define BLE_RESET_GPIO_Port GPIOB
#define BLE_SPI_CS_Pin GPIO_PIN_12
#define BLE_SPI_CS_GPIO_Port GPIOB
#define BLE_SPI_IRQ_Pin GPIO_PIN_13
#define BLE_SPI_IRQ_GPIO_Port GPIOB
#define BLE_SPI_IRQ_EXTI_IRQn EXTI4_15_IRQn
#define USR_BTN_Pin GPIO_PIN_14
#define USR_BTN_GPIO_Port GPIOB
#define USR_BTN_EXTI_IRQn EXTI4_15_IRQn
#define CAN_PWR_CLK_Pin GPIO_PIN_3
#define CAN_PWR_CLK_GPIO_Port GPIOB
#define CAN_PWR_EN_Pin GPIO_PIN_4
#define CAN_PWR_EN_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
