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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Btn_Pin GPIO_PIN_2
#define Btn_GPIO_Port GPIOE
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define M1_MBL_Pin GPIO_PIN_4
#define M1_MBL_GPIO_Port GPIOE
#define DAH_Pin GPIO_PIN_5
#define DAH_GPIO_Port GPIOE
#define M1_MAL_Pin GPIO_PIN_6
#define M1_MAL_GPIO_Port GPIOE
#define M0_MBL_Pin GPIO_PIN_13
#define M0_MBL_GPIO_Port GPIOC
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define BATREF_Pin GPIO_PIN_0
#define BATREF_GPIO_Port GPIOC
#define M0_CUR_Pin GPIO_PIN_1
#define M0_CUR_GPIO_Port GPIOC
#define M2_MAL_Pin GPIO_PIN_2
#define M2_MAL_GPIO_Port GPIOC
#define M2_ENCA_Pin GPIO_PIN_0
#define M2_ENCA_GPIO_Port GPIOA
#define M2_ENCB_Pin GPIO_PIN_1
#define M2_ENCB_GPIO_Port GPIOA
#define M1_CUR_Pin GPIO_PIN_2
#define M1_CUR_GPIO_Port GPIOA
#define M3_CUR_Pin GPIO_PIN_3
#define M3_CUR_GPIO_Port GPIOA
#define D_CUR_Pin GPIO_PIN_4
#define D_CUR_GPIO_Port GPIOA
#define M2_CUR_Pin GPIO_PIN_4
#define M2_CUR_GPIO_Port GPIOC
#define nRF_IRQ_Pin GPIO_PIN_5
#define nRF_IRQ_GPIO_Port GPIOC
#define nRF_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define CHARGE_EN_Pin GPIO_PIN_0
#define CHARGE_EN_GPIO_Port GPIOB
#define M2_MBL_Pin GPIO_PIN_1
#define M2_MBL_GPIO_Port GPIOB
#define M2_MBH_Pin GPIO_PIN_11
#define M2_MBH_GPIO_Port GPIOE
#define M3_MBH_Pin GPIO_PIN_13
#define M3_MBH_GPIO_Port GPIOE
#define M3_MAH_Pin GPIO_PIN_14
#define M3_MAH_GPIO_Port GPIOE
#define TX_Detect_Pin GPIO_PIN_15
#define TX_Detect_GPIO_Port GPIOE
#define M3_MBL_Pin GPIO_PIN_11
#define M3_MBL_GPIO_Port GPIOB
#define M3_MAL_Pin GPIO_PIN_12
#define M3_MAL_GPIO_Port GPIOB
#define CHIP_KICK_Pin GPIO_PIN_8
#define CHIP_KICK_GPIO_Port GPIOD
#define KICK_Pin GPIO_PIN_10
#define KICK_GPIO_Port GPIOD
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define M2_MAH_Pin GPIO_PIN_6
#define M2_MAH_GPIO_Port GPIOC
#define M0_MBH_Pin GPIO_PIN_7
#define M0_MBH_GPIO_Port GPIOC
#define M1_MBH_Pin GPIO_PIN_8
#define M1_MBH_GPIO_Port GPIOC
#define M0_MAH_Pin GPIO_PIN_9
#define M0_MAH_GPIO_Port GPIOC
#define M1__MAH_Pin GPIO_PIN_8
#define M1__MAH_GPIO_Port GPIOA
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define M1_ENCA_Pin GPIO_PIN_15
#define M1_ENCA_GPIO_Port GPIOA
#define nRF_CE_Pin GPIO_PIN_12
#define nRF_CE_GPIO_Port GPIOC
#define nRF_CSn_Pin GPIO_PIN_0
#define nRF_CSn_GPIO_Port GPIOD
#define S1_Pin GPIO_PIN_1
#define S1_GPIO_Port GPIOD
#define USD_CS_Pin GPIO_PIN_3
#define USD_CS_GPIO_Port GPIOD
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define M0_MAL_Pin GPIO_PIN_7
#define M0_MAL_GPIO_Port GPIOD
#define M1_ENCB_Pin GPIO_PIN_3
#define M1_ENCB_GPIO_Port GPIOB
#define M0_ENCA_Pin GPIO_PIN_4
#define M0_ENCA_GPIO_Port GPIOB
#define M0_ENCB_Pin GPIO_PIN_5
#define M0_ENCB_GPIO_Port GPIOB
#define M3_ENCA_Pin GPIO_PIN_6
#define M3_ENCA_GPIO_Port GPIOB
#define M3_ENCB_Pin GPIO_PIN_7
#define M3_ENCB_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
