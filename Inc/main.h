/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define Line_V_P_Pin GPIO_PIN_0
#define Line_V_P_GPIO_Port GPIOC
#define Line_V_N_Pin GPIO_PIN_1
#define Line_V_N_GPIO_Port GPIOC
#define DC_Bus_Pin GPIO_PIN_2
#define DC_Bus_GPIO_Port GPIOC
#define DAC_Out_Pin GPIO_PIN_4
#define DAC_Out_GPIO_Port GPIOA
#define Ph_AN_PWM_Pin GPIO_PIN_7
#define Ph_AN_PWM_GPIO_Port GPIOA
#define Ph_BN_PWM_Pin GPIO_PIN_0
#define Ph_BN_PWM_GPIO_Port GPIOB
#define Ph_A_PWM_Pin GPIO_PIN_9
#define Ph_A_PWM_GPIO_Port GPIOE
#define Ph_B_PWM_Pin GPIO_PIN_11
#define Ph_B_PWM_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
