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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define HALL8_Pin GPIO_PIN_0
#define HALL8_GPIO_Port GPIOA
#define HALL7_Pin GPIO_PIN_1
#define HALL7_GPIO_Port GPIOA
#define HALL6_Pin GPIO_PIN_2
#define HALL6_GPIO_Port GPIOA
#define HALL5_Pin GPIO_PIN_3
#define HALL5_GPIO_Port GPIOA
#define HALL4_Pin GPIO_PIN_4
#define HALL4_GPIO_Port GPIOA
#define HALL3_Pin GPIO_PIN_5
#define HALL3_GPIO_Port GPIOA
#define HALL2_Pin GPIO_PIN_6
#define HALL2_GPIO_Port GPIOA
#define HALL1_Pin GPIO_PIN_7
#define HALL1_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_0
#define PWM1_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_1
#define PWM2_GPIO_Port GPIOB
#define PWM3_Pin GPIO_PIN_2
#define PWM3_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_10
#define PWM4_GPIO_Port GPIOB
#define OTW1_Pin GPIO_PIN_11
#define OTW1_GPIO_Port GPIOB
#define FAULT1_Pin GPIO_PIN_12
#define FAULT1_GPIO_Port GPIOB
#define PWM_A1_Pin GPIO_PIN_13
#define PWM_A1_GPIO_Port GPIOB
#define RESET_AB1_Pin GPIO_PIN_14
#define RESET_AB1_GPIO_Port GPIOB
#define PWM_B1_Pin GPIO_PIN_15
#define PWM_B1_GPIO_Port GPIOB
#define PWM_C1_Pin GPIO_PIN_8
#define PWM_C1_GPIO_Port GPIOA
#define RESET_CD1_Pin GPIO_PIN_11
#define RESET_CD1_GPIO_Port GPIOA
#define PWM_D1_Pin GPIO_PIN_12
#define PWM_D1_GPIO_Port GPIOA
#define OTW2_Pin GPIO_PIN_15
#define OTW2_GPIO_Port GPIOA
#define FAULT2_Pin GPIO_PIN_3
#define FAULT2_GPIO_Port GPIOB
#define PWM_A2_Pin GPIO_PIN_4
#define PWM_A2_GPIO_Port GPIOB
#define RESET_AB2_Pin GPIO_PIN_5
#define RESET_AB2_GPIO_Port GPIOB
#define PWM_B2_Pin GPIO_PIN_6
#define PWM_B2_GPIO_Port GPIOB
#define PWM_C2_Pin GPIO_PIN_7
#define PWM_C2_GPIO_Port GPIOB
#define RESET_CD2_Pin GPIO_PIN_8
#define RESET_CD2_GPIO_Port GPIOB
#define PWM_D2_Pin GPIO_PIN_9
#define PWM_D2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define hall_bufsize 8
#define motor_cnt 4

#define speed_max 1000

extern int step_old[4];
extern int speed[4];
extern int pwm[4];
extern int step[4];
extern int status[4];

extern int setspeed[4];

extern int hall[8];
extern int hall_old[8];
extern int hall_tmp[8][hall_bufsize];
extern int hall_cnt;

extern int servo[4];
extern int servo_tmp[4];
extern int servo_old[4];

extern uint16_t hall_pin[8];
extern GPIO_TypeDef *hall_port[8];
extern uint16_t pwm_pin[8];
extern GPIO_TypeDef *pwm_port[8];
extern uint16_t servo_pin[4];
extern GPIO_TypeDef *servo_port[4];

extern int us_per_timer1;
extern int dir[4];
extern int counter_control;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
