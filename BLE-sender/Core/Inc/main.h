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
#include "stm32l4xx_hal.h"

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
#define SDMMC1_CDIR_Pin GPIO_PIN_9
#define SDMMC1_CDIR_GPIO_Port GPIOB
#define SPI3_MISO_Pin GPIO_PIN_4
#define SPI3_MISO_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_3
#define SPI2_MISO_GPIO_Port GPIOD
#define SDMMC1_D3_Pin GPIO_PIN_11
#define SDMMC1_D3_GPIO_Port GPIOC
#define INT2_LSM6DSOX_Pin GPIO_PIN_3
#define INT2_LSM6DSOX_GPIO_Port GPIOE
#define SDMMC1_CKIN_Pin GPIO_PIN_8
#define SDMMC1_CKIN_GPIO_Port GPIOB
#define SPI3_MOSI_Pin GPIO_PIN_5
#define SPI3_MOSI_GPIO_Port GPIOB
#define SPI3_SCK_Pin GPIO_PIN_3
#define SPI3_SCK_GPIO_Port GPIOB
#define USART2_RX_Pin GPIO_PIN_6
#define USART2_RX_GPIO_Port GPIOD
#define SPI2_SCK_Pin GPIO_PIN_1
#define SPI2_SCK_GPIO_Port GPIOD
#define CS_LIS2MDL_Pin GPIO_PIN_15
#define CS_LIS2MDL_GPIO_Port GPIOA
#define USB_OTG_DP_Pin GPIO_PIN_12
#define USB_OTG_DP_GPIO_Port GPIOA
#define USB_OTG_DM_Pin GPIO_PIN_11
#define USB_OTG_DM_GPIO_Port GPIOA
#define SD_SEL_Pin GPIO_PIN_5
#define SD_SEL_GPIO_Port GPIOE
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define SPI2_INT_Pin GPIO_PIN_4
#define SPI2_INT_GPIO_Port GPIOD
#define SPI2_INT_EXTI_IRQn EXTI4_IRQn
#define SPI2_CS_Pin GPIO_PIN_0
#define SPI2_CS_GPIO_Port GPIOD
#define SDMMC1_D2_Pin GPIO_PIN_10
#define SDMMC1_D2_GPIO_Port GPIOC
#define SDMMC1_D1_Pin GPIO_PIN_9
#define SDMMC1_D1_GPIO_Port GPIOC
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define SD_EN_Pin GPIO_PIN_4
#define SD_EN_GPIO_Port GPIOE
#define INT2_IIS3DHHC_Pin GPIO_PIN_6
#define INT2_IIS3DHHC_GPIO_Port GPIOE
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define USART2_TX_Pin GPIO_PIN_5
#define USART2_TX_GPIO_Port GPIOD
#define SDMMC1_CMD_Pin GPIO_PIN_2
#define SDMMC1_CMD_GPIO_Port GPIOD
#define SDMMC1_CK_Pin GPIO_PIN_12
#define SDMMC1_CK_GPIO_Port GPIOC
#define BLE_RESET_Pin GPIO_PIN_8
#define BLE_RESET_GPIO_Port GPIOA
#define SDMMC1_D0DIR_Pin GPIO_PIN_6
#define SDMMC1_D0DIR_GPIO_Port GPIOC
#define USER_LED_Pin GPIO_PIN_2
#define USER_LED_GPIO_Port GPIOF
#define BB_MODE_Pin GPIO_PIN_0
#define BB_MODE_GPIO_Port GPIOF
#define CHG_LED2_Pin GPIO_PIN_10
#define CHG_LED2_GPIO_Port GPIOG
#define I2C3_SDA_Pin GPIO_PIN_8
#define I2C3_SDA_GPIO_Port GPIOG
#define SDMMC1_D123DIR_Pin GPIO_PIN_7
#define SDMMC1_D123DIR_GPIO_Port GPIOC
#define SDMMC1_D0_Pin GPIO_PIN_8
#define SDMMC1_D0_GPIO_Port GPIOC
#define I2C3_SCL_Pin GPIO_PIN_7
#define I2C3_SCL_GPIO_Port GPIOG
#define LDO_2V7_EN_Pin GPIO_PIN_7
#define LDO_2V7_EN_GPIO_Port GPIOF
#define CHG_LED_Pin GPIO_PIN_3
#define CHG_LED_GPIO_Port GPIOG
#define SW_SEL_Pin GPIO_PIN_4
#define SW_SEL_GPIO_Port GPIOG
#define INT_LPS22HH_Pin GPIO_PIN_15
#define INT_LPS22HH_GPIO_Port GPIOD
#define INT2_LIS2DW12_Pin GPIO_PIN_14
#define INT2_LIS2DW12_GPIO_Port GPIOD
#define INT_HTS221_Pin GPIO_PIN_13
#define INT_HTS221_GPIO_Port GPIOD
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOH
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOH
#define CS_LIS2DW12_Pin GPIO_PIN_11
#define CS_LIS2DW12_GPIO_Port GPIOE
#define SPI2_MOSI_Pin GPIO_PIN_3
#define SPI2_MOSI_GPIO_Port GPIOC
#define SPI2_SCKE13_Pin GPIO_PIN_13
#define SPI2_SCKE13_GPIO_Port GPIOE
#define CPU_LED_Pin GPIO_PIN_15
#define CPU_LED_GPIO_Port GPIOB
#define INT1_LIS2DW12_Pin GPIO_PIN_5
#define INT1_LIS2DW12_GPIO_Port GPIOC
#define SPI1_MOSI_Pin GPIO_PIN_15
#define SPI1_MOSI_GPIO_Port GPIOE
#define INT1_LSM6DSOX_Pin GPIO_PIN_2
#define INT1_LSM6DSOX_GPIO_Port GPIOA
#define CS_IIS3DHHC_Pin GPIO_PIN_10
#define CS_IIS3DHHC_GPIO_Port GPIOE
#define SD_DETECT_Pin GPIO_PIN_12
#define SD_DETECT_GPIO_Port GPIOB
#define USER_PB1_Pin GPIO_PIN_1
#define USER_PB1_GPIO_Port GPIOG
#define CS_LSM6DSOX_Pin GPIO_PIN_12
#define CS_LSM6DSOX_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
