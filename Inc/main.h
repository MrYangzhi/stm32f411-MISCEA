/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define TIM5_CH1_IN1_Pin GPIO_PIN_0
#define TIM5_CH1_IN1_GPIO_Port GPIOA
#define TIM2_CH2_PWM1_Pin GPIO_PIN_1
#define TIM2_CH2_PWM1_GPIO_Port GPIOA
#define TIM5_CH3_IN2_Pin GPIO_PIN_2
#define TIM5_CH3_IN2_GPIO_Port GPIOA
#define TIM2_CH4_PWM2_Pin GPIO_PIN_3
#define TIM2_CH4_PWM2_GPIO_Port GPIOA
#define LED6_Pin GPIO_PIN_4
#define LED6_GPIO_Port GPIOA
#define TIM2_CH1_PWM3_Pin GPIO_PIN_5
#define TIM2_CH1_PWM3_GPIO_Port GPIOA
#define LED7_Pin GPIO_PIN_6
#define LED7_GPIO_Port GPIOA
#define LED12_Pin GPIO_PIN_7
#define LED12_GPIO_Port GPIOA
#define LED13_Pin GPIO_PIN_0
#define LED13_GPIO_Port GPIOB
#define TIM3_CH4_IN3_Pin GPIO_PIN_1
#define TIM3_CH4_IN3_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_8
#define LED8_GPIO_Port GPIOA
#define TIM3_CH1_IN4_Pin GPIO_PIN_4
#define TIM3_CH1_IN4_GPIO_Port GPIOB
#define TIM4_CH1_PWM4_Pin GPIO_PIN_6
#define TIM4_CH1_PWM4_GPIO_Port GPIOB
#define TIM10_CH1_PWM5_Pin GPIO_PIN_8
#define TIM10_CH1_PWM5_GPIO_Port GPIOB
#define TIM11_CH1_IN5_Pin GPIO_PIN_9
#define TIM11_CH1_IN5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
