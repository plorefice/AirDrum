/**
  ******************************************************************************
  * @file    Templates/Src/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.0 modified by ARM
  * @date    26-June-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

#ifdef _RTE_
#include "RTE_Components.h"             // Component selection
#endif

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern int16_t 							IMU_L_Buffer[3];
extern void 								MIDI_SendMsg (uint8_t type, uint8_t note, uint8_t velocity);
extern IMU_Click_Detection	IMU_L_Detection;
extern IMU_Click_Detection	IMU_R_Detection;
uint8_t											th_X, th_Y, th_Z;
uint16_t										max_X, max_Y, max_Z;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void ClickDetection(IMU_Click_Detection *IMU, int16_t * buffer)
{
		int16_t x = buffer[0];
		int16_t y = buffer[1];
		int16_t z = buffer[2];
		
		if(th_X){
			if(x < TH) {
				
				if(max_X > MAX_FORCE)
					max_X = MAX_FORCE;
				IMU->x.vel  = (max_X/MAX_FORCE) * 127;
				IMU->x.click = 1;
				th_X = 0;
				max_X = 0;
			} else if(x > max_X) {
				max_X = x;
			}
		} else if(x > TH){
			max_X = x;
			th_X = 1;
		}
		
		if(th_Y && y < TH){
			
			if(max_Y > MAX_FORCE)
				max_Y = MAX_FORCE;
			IMU->y.vel = (max_Y/MAX_FORCE) * 127;
			IMU->y.click = 1;
			th_Y = 0;
		} else if(y > TH){
			max_Y = y;
			th_Y = 1;
		}
		
		if(th_Z && z < TH){
				
			if(max_Z > MAX_FORCE)
				max_Z = MAX_FORCE;
			IMU->z.vel = (max_Z/MAX_FORCE) * 127;
			IMU->z.click = 1;	
			th_Z = 0;
		} else if(z > TH){
			max_Z = z;
			th_Z = 1;
		}
}
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
#ifndef RTE_CMSIS_RTOS_RTX
void SVC_Handler(void)
{
}
#endif

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
#ifndef RTE_CMSIS_RTOS_RTX
void PendSV_Handler(void)
{
}
#endif

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
#ifndef RTE_CMSIS_RTOS_RTX
void SysTick_Handler(void)
{
  HAL_IncTick();
}
#endif

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

#ifdef IMU_L
void IMU_L_IRQ_Handler (void)
{
	if (__HAL_GPIO_EXTI_GET_IT (IMU_L_IRQ_PIN))
	{
		HAL_GPIO_TogglePin (LED_IRQ_L_PORT, LED_IRQ_L_PIN);
		
		MPU9150_ReadGyro (&IMU_L_Handler, IMU_L_Buffer);
		ClickDetection(&IMU_L_Detection, IMU_L_Buffer);
		__HAL_GPIO_EXTI_CLEAR_IT (IMU_L_IRQ_PIN);
	}
}
#endif

#ifdef IMU_R
void IMU_R_IRQ_Handler (void)
{
	if (__HAL_GPIO_EXTI_GET_IT (IMU_R_IRQ_PIN))
	{
		HAL_GPIO_TogglePin (LED_IRQ_R_PORT, LED_IRQ_R_PIN);
		
		
		__HAL_GPIO_EXTI_CLEAR_IT (IMU_R_IRQ_PIN);
	}
}
#endif


/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
