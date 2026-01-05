/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
  struct Struct_Run
  {
    uint8_t Time2_Count;
    float time;
    uint8_t Runflag;
  };

  struct Struct_Rx
  {
    uint8_t flag;
    uint8_t data[40];
    uint8_t process_flag;
  };

  extern struct Struct_Run Run;
  extern struct Struct_Rx rx;

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
#define key1_Pin GPIO_PIN_13
#define key1_GPIO_Port GPIOE
#define key2_Pin GPIO_PIN_14
#define key2_GPIO_Port GPIOE
#define key3_Pin GPIO_PIN_15
#define key3_GPIO_Port GPIOE
#define led1_Pin GPIO_PIN_6
#define led1_GPIO_Port GPIOB
#define led2_Pin GPIO_PIN_7
#define led2_GPIO_Port GPIOB
#define led3_Pin GPIO_PIN_8
#define led3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
