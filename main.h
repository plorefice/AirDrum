/**
  ******************************************************************************
  * @file    Templates/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "mpu9150_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define IMU_L
//#define IMU_R

#define IMU_GYRO_THRESHOLD                 0x07FF
#define IMU_MAX_FORCE                      32767.0f

/* Left IMU configuration */
#define IMU_L_I2C                          I2C1
#define IMU_L_I2C_ENABLE                   __I2C1_CLK_ENABLE
#define IMU_L_I2C_AF                       GPIO_AF4_I2C1

#define IMU_L_I2C_SCL_ENABLE               __GPIOB_CLK_ENABLE
#define IMU_L_I2C_SCL_PORT                 GPIOB
#define IMU_L_I2C_SCL_PIN                  GPIO_PIN_6

#define IMU_L_I2C_SDA_ENABLE               __GPIOB_CLK_ENABLE
#define IMU_L_I2C_SDA_PORT                 GPIOB
#define IMU_L_I2C_SDA_PIN                  GPIO_PIN_7

#define IMU_L_IRQ_ENABLE                   __GPIOB_CLK_ENABLE
#define IMU_L_IRQ_PORT                     GPIOB
#define IMU_L_IRQ_PIN                      GPIO_PIN_4
#define IMU_L_IRQ_LEVEL                    GPIO_MODE_IT_RISING

#define IMU_L_IRQ_EXTI                     EXTI4_IRQn
#define IMU_L_IRQ_Handler                  EXTI4_IRQHandler

/* Right IMU configuration */
#define IMU_R_I2C                          I2C3
#define IMU_R_I2C_ENABLE                   __I2C2_CLK_ENABLE
#define IMU_R_I2C_AF                       GPIO_AF4_I2C2

#define IMU_R_I2C_SCL_ENABLE               __GPIOB_CLK_ENABLE
#define IMU_R_I2C_SCL_PORT                 GPIOB
#define IMU_R_I2C_SCL_PIN                  GPIO_PIN_10

#define IMU_R_I2C_SDA_ENABLE               __GPIOB_CLK_ENABLE
#define IMU_R_I2C_SDA_PORT                 GPIOB
#define IMU_R_I2C_SDA_PIN                  GPIO_PIN_11

#define IMU_R_IRQ_ENABLE                   __GPIOB_CLK_ENABLE
#define IMU_R_IRQ_PORT                     GPIOB
#define IMU_R_IRQ_PIN                      GPIO_PIN_5
#define IMU_R_IRQ_LEVEL                    GPIO_MODE_IT_RISING

#define IMU_R_IRQ_EXTI                     EXTI9_5_IRQn
#define IMU_R_IRQ_Handler                  EXTI9_5_IRQHandler

/* Common IMU configuration */
#define IMU_LP_FILTER                      MPU9150_LOWPASSFILTER_1

/* USB configuration */
#define USB_Device                         0

/* Left IMU IRQ LED configuration */
#define LED_IRQ_L_CLK_ENABLE               __GPIOD_CLK_ENABLE
#define LED_IRQ_L_PORT                     GPIOD
#define LED_IRQ_L_PIN                      GPIO_PIN_12

/* Right IMU IRQ LED configuration */
#define LED_IRQ_R_CLK_ENABLE               __GPIOD_CLK_ENABLE
#define LED_IRQ_R_PORT                     GPIOD
#define LED_IRQ_R_PIN                      GPIO_PIN_13

/* Error LED configuration */
#define LED_ERR_CLK_ENABLE                 __GPIOD_CLK_ENABLE
#define LED_ERR_PORT                       GPIOD
#define LED_ERR_PIN                        GPIO_PIN_14

/* MIDI LED configuration */
#define LED_MIDI_CLK_ENABLE                __GPIOD_CLK_ENABLE
#define LED_MIDI_PORT                      GPIOD
#define LED_MIDI_PIN                       GPIO_PIN_15

/* Exported macro ------------------------------------------------------------*/
#define IMU_SMPLRT(X) (uint8_t)((IMU_LP_FILTER == MPU9150_LOWPASSFILTER_0) ? \
                               ((8000 - X) / X) : ((1000 - X) / X))

/* Exported variables --------------------------------------------------------*/
extern I2C_HandleTypeDef                   IMU_L_I2C_Handler;
extern I2C_HandleTypeDef                   IMU_R_I2C_Handler;

extern MPU9150_HandleTypeDef               IMU_L_Handler;
extern MPU9150_HandleTypeDef               IMU_R_Handler;

/* Exported functions ------------------------------------------------------- */
void Error_Handler(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
