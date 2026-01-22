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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Motor5_F_Pin GPIO_PIN_2
#define Motor5_F_GPIO_Port GPIOC
#define Motor5_R_Pin GPIO_PIN_3
#define Motor5_R_GPIO_Port GPIOC
#define Motor3_F_Pin GPIO_PIN_4
#define Motor3_F_GPIO_Port GPIOC
#define Motor3_R_Pin GPIO_PIN_5
#define Motor3_R_GPIO_Port GPIOC
#define Motor1_F_Pin GPIO_PIN_12
#define Motor1_F_GPIO_Port GPIOE
#define Motor1_R_Pin GPIO_PIN_13
#define Motor1_R_GPIO_Port GPIOE
#define Moto8_R_Pin GPIO_PIN_12
#define Moto8_R_GPIO_Port GPIOB
#define Motor8_F_Pin GPIO_PIN_13
#define Motor8_F_GPIO_Port GPIOB
#define Motor4_F_Pin GPIO_PIN_10
#define Motor4_F_GPIO_Port GPIOD
#define Motor4_R_Pin GPIO_PIN_11
#define Motor4_R_GPIO_Port GPIOD
#define Motor6_F_Pin GPIO_PIN_8
#define Motor6_F_GPIO_Port GPIOC
#define Motor6_R_Pin GPIO_PIN_9
#define Motor6_R_GPIO_Port GPIOC
#define RS485_DE_RE_Pin GPIO_PIN_8
#define RS485_DE_RE_GPIO_Port GPIOA
#define Motor2_F_Pin GPIO_PIN_4
#define Motor2_F_GPIO_Port GPIOB
#define Motor2_R_Pin GPIO_PIN_5
#define Motor2_R_GPIO_Port GPIOB
#define Motor9_F_Pin GPIO_PIN_7
#define Motor9_F_GPIO_Port GPIOB
#define Motor9_R_Pin GPIO_PIN_6
#define Motor9_R_GPIO_Port GPIOB
#define Motor7_R_Pin GPIO_PIN_1
#define Motor7_R_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
/* Flash Memory Address for Slave ID Storage */
/* STM32F407VETX: 512KB Flash (0x08000000 - 0x0807FFFF), using last 4 bytes */
#define SLAVE_ID_FLASH_ADDRESS 0x0807FFFC  /* Last 4 bytes of Flash */
#define SLAVE_ID_MAGIC         0xABCD1234   /* Magic number to verify valid ID */
#define SLAVE_ID_DEFAULT       0            /* Default ID if Flash is empty */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
