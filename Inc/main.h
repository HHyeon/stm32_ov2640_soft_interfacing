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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD_GREEN_Pin GPIO_PIN_13
#define LD_GREEN_GPIO_Port GPIOC
#define P0_Pin GPIO_PIN_0
#define P0_GPIO_Port GPIOA
#define P1_Pin GPIO_PIN_1
#define P1_GPIO_Port GPIOA
#define P2_Pin GPIO_PIN_2
#define P2_GPIO_Port GPIOA
#define P3_Pin GPIO_PIN_3
#define P3_GPIO_Port GPIOA
#define P4_Pin GPIO_PIN_4
#define P4_GPIO_Port GPIOA
#define P5_Pin GPIO_PIN_5
#define P5_GPIO_Port GPIOA
#define P6_Pin GPIO_PIN_6
#define P6_GPIO_Port GPIOA
#define P7_Pin GPIO_PIN_7
#define P7_GPIO_Port GPIOA
#define XCLK_Pin GPIO_PIN_1
#define XCLK_GPIO_Port GPIOB
#define PCLK_Pin GPIO_PIN_12
#define PCLK_GPIO_Port GPIOB
#define HREF_Pin GPIO_PIN_13
#define HREF_GPIO_Port GPIOB
#define VSYNC_Pin GPIO_PIN_14
#define VSYNC_GPIO_Port GPIOB
#define CAM_PWDN_Pin GPIO_PIN_6
#define CAM_PWDN_GPIO_Port GPIOB
#define CAM_RET_Pin GPIO_PIN_7
#define CAM_RET_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define LD_GREEN_LOW HAL_GPIO_WritePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin, GPIO_PIN_RESET)
#define LD_GREEN_HIGH HAL_GPIO_WritePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin, GPIO_PIN_SET)

#define CAM_PWDN_LOW HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_RESET)
#define CAM_PWDN_HIGH HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_SET)

#define CAM_RET_LOW HAL_GPIO_WritePin(CAM_RET_GPIO_Port, CAM_RET_Pin, GPIO_PIN_RESET)
#define CAM_RET_HIGH HAL_GPIO_WritePin(CAM_RET_GPIO_Port, CAM_RET_Pin, GPIO_PIN_SET)

#define OV2640_DSP_ENABLE ov2640_write_byte(BANK_SEL, BANK_SEL_DSP) // 0
#define OV2640_SEN_ENABLE ov2640_write_byte(BANK_SEL, BANK_SEL_SENS) // 1
  
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
