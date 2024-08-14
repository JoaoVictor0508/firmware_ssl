/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
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
int init(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TMR11_CLK 84000000
#define DATA_Ready_Pin GPIO_PIN_2
#define DATA_Ready_GPIO_Port GPIOE
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define INT1_Pin GPIO_PIN_4
#define INT1_GPIO_Port GPIOE
#define INT2_Pin GPIO_PIN_5
#define INT2_GPIO_Port GPIOE
#define PWM_M2_Pin GPIO_PIN_6
#define PWM_M2_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define BATTERY_LVL_Pin GPIO_PIN_1
#define BATTERY_LVL_GPIO_Port GPIOC
#define PWM_BUZZER_Pin GPIO_PIN_3
#define PWM_BUZZER_GPIO_Port GPIOC
#define USER_BUTTON_Pin GPIO_PIN_0
#define USER_BUTTON_GPIO_Port GPIOA
#define PWM_M3_Pin GPIO_PIN_1
#define PWM_M3_GPIO_Port GPIOA
#define PWM_ROLLER_Pin GPIO_PIN_2
#define PWM_ROLLER_GPIO_Port GPIOA
#define PWM_M4_Pin GPIO_PIN_3
#define PWM_M4_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define V_CAP_Pin GPIO_PIN_4
#define V_CAP_GPIO_Port GPIOC
#define RIGHT_KICK_Pin GPIO_PIN_0
#define RIGHT_KICK_GPIO_Port GPIOB
#define CHIP_KICK_Pin GPIO_PIN_1
#define CHIP_KICK_GPIO_Port GPIOB
#define LEFT_KICK_Pin GPIO_PIN_2
#define LEFT_KICK_GPIO_Port GPIOB
#define SHOOT_EN_Pin GPIO_PIN_8
#define SHOOT_EN_GPIO_Port GPIOE
#define ENC_M3_A_Pin GPIO_PIN_9
#define ENC_M3_A_GPIO_Port GPIOE
#define ENC_M3_B_Pin GPIO_PIN_11
#define ENC_M3_B_GPIO_Port GPIOE
#define SPI2_SS_Pin GPIO_PIN_12
#define SPI2_SS_GPIO_Port GPIOB
#define ROBOT_ID_0_Pin GPIO_PIN_8
#define ROBOT_ID_0_GPIO_Port GPIOD
#define ROBOT_ID_1_Pin GPIO_PIN_9
#define ROBOT_ID_1_GPIO_Port GPIOD
#define ROBOT_ID_2_Pin GPIO_PIN_10
#define ROBOT_ID_2_GPIO_Port GPIOD
#define ENC_M4_A_Pin GPIO_PIN_12
#define ENC_M4_A_GPIO_Port GPIOD
#define ENC_M4_B_Pin GPIO_PIN_13
#define ENC_M4_B_GPIO_Port GPIOD
#define ROBOT_ID_3_Pin GPIO_PIN_14
#define ROBOT_ID_3_GPIO_Port GPIOD
#define ROBOT_CH_Pin GPIO_PIN_15
#define ROBOT_CH_GPIO_Port GPIOD
#define ENC_M1_A_Pin GPIO_PIN_6
#define ENC_M1_A_GPIO_Port GPIOC
#define ENC_M1_B_Pin GPIO_PIN_7
#define ENC_M1_B_GPIO_Port GPIOC
#define RADIO_SS_Pin GPIO_PIN_8
#define RADIO_SS_GPIO_Port GPIOC
#define RADIO_CS_Pin GPIO_PIN_9
#define RADIO_CS_GPIO_Port GPIOC
#define BALL_SENSOR_Pin GPIO_PIN_8
#define BALL_SENSOR_GPIO_Port GPIOA
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
#define ENC_M2_A_Pin GPIO_PIN_15
#define ENC_M2_A_GPIO_Port GPIOA
#define RADIO_SCK_Pin GPIO_PIN_10
#define RADIO_SCK_GPIO_Port GPIOC
#define RADIO_MISO_Pin GPIO_PIN_11
#define RADIO_MISO_GPIO_Port GPIOC
#define RADIO_MOSI_Pin GPIO_PIN_12
#define RADIO_MOSI_GPIO_Port GPIOC
#define RADIO_IRQ_Pin GPIO_PIN_0
#define RADIO_IRQ_GPIO_Port GPIOD
#define RADIO_IRQ_EXTI_IRQn EXTI0_IRQn
#define CHARGE_EN_Pin GPIO_PIN_1
#define CHARGE_EN_GPIO_Port GPIOD
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define ENC_M2_B_Pin GPIO_PIN_3
#define ENC_M2_B_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_4
#define GREEN_LED_GPIO_Port GPIOB
#define RED_LED_Pin GPIO_PIN_5
#define RED_LED_GPIO_Port GPIOB
#define SCL_ACCELEROMETER_Pin GPIO_PIN_6
#define SCL_ACCELEROMETER_GPIO_Port GPIOB
#define ORANGE_LED_Pin GPIO_PIN_7
#define ORANGE_LED_GPIO_Port GPIOB
#define PWM_M1_Pin GPIO_PIN_8
#define PWM_M1_GPIO_Port GPIOB
#define SDA_ACCELEROMETER_Pin GPIO_PIN_9
#define SDA_ACCELEROMETER_GPIO_Port GPIOB
#define BLUE_LED_Pin GPIO_PIN_0
#define BLUE_LED_GPIO_Port GPIOE
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define M_PI 3.14159265358979323846264338328 // Pi

#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b;       \
})

#define min(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b;       \
})
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
