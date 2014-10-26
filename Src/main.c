/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 23/10/2014 23:16:15
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
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
#include "cmsis_os.h"
#include "usb_device.h"
#include "stm32f4xx_it.h"

#include "inv_gyro.h"
#include "inv_gyro_dmp_android.h"
#include "mpu9150_hal.h"
#include "midi.h"

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef                   IMU_L_I2C_Handler;
I2C_HandleTypeDef                   IMU_R_I2C_Handler;

MPU9150_HandleTypeDef               IMU_L_Handler;
MPU9150_HandleTypeDef               IMU_R_Handler;

/* Private function prototypes -----------------------------------------------*/
       void SystemClock_Config (void);
			 void Error_Handler      (void);
static void MIDI_Thread        (void const * argument);

static void MX_GPIO_Init       (void);
static void MX_I2C_Init        (void);
static void MX_IMU_Init        (void);

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	SystemCoreClockUpdate();

  /* System interrupt init*/
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_InitTick(0);
  
  /* Initialize all configured peripherals */
  MX_I2C_Init ();
  MX_GPIO_Init ();
	MX_IMU_Init ();
	
  /* Code generated for FreeRTOS */
  /* Create Start thread */
  osThreadDef(MIDI_Thread, MIDI_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(MIDI_Thread), NULL);

  /* Start scheduler */
  osKernelStart(NULL, NULL);

  while (1)
		;
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

}


void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /* Configure push-button */
  GPIO_InitStruct.Pin   = GPIO_PIN_0;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
	/* Configure LED MIDI */
  LED_MIDI_CLK_ENABLE ();
  
  GPIO_InitStruct.Pin   = LED_MIDI_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(LED_MIDI_PORT, &GPIO_InitStruct);
	
	/* Configure LED ERR */
  LED_ERR_CLK_ENABLE ();
  
  GPIO_InitStruct.Pin   = LED_ERR_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(LED_ERR_PORT, &GPIO_InitStruct);

#ifdef IMU_L  
  /* IRQ pin configuration */
  IMU_L_IRQ_ENABLE ();
  
  GPIO_InitStruct.Pin    = IMU_L_IRQ_PIN;
  GPIO_InitStruct.Mode   = IMU_L_IRQ_LEVEL;
  GPIO_InitStruct.Speed  = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init (IMU_L_IRQ_PORT, &GPIO_InitStruct);
  
  HAL_NVIC_SetPriority (IMU_L_IRQ_EXTI, 1, 0);
  HAL_NVIC_EnableIRQ (IMU_L_IRQ_EXTI);
  
  /* IRQ led configuration */
  LED_IRQ_L_CLK_ENABLE ();
  
  GPIO_InitStruct.Pin    = LED_IRQ_L_PIN;
  GPIO_InitStruct.Mode   = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull   = GPIO_NOPULL;
  HAL_GPIO_Init (LED_IRQ_L_PORT, &GPIO_InitStruct);
#endif
  
#ifdef IMU_R
  /* IRQ pin configuration */
  IMU_R_IRQ_ENABLE ();
  
  GPIO_InitStruct.Pin    = IMU_R_IRQ_PIN;
  GPIO_InitStruct.Mode   = IMU_R_IRQ_LEVEL;
  GPIO_InitStruct.Speed  = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init (IMU_R_IRQ_PORT, &GPIO_InitStruct);
  
  HAL_NVIC_SetPriority (IMU_R_IRQ_EXTI, 1, 0);
  HAL_NVIC_EnableIRQ (IMU_R_IRQ_EXTI);
  
  /* IRQ led configuration */
  LED_IRQ_R_CLK_ENABLE ();
  
  GPIO_InitStruct.Pin    = LED_IRQ_R_PIN;
  GPIO_InitStruct.Mode   = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull   = GPIO_NOPULL;
  HAL_GPIO_Init (LED_IRQ_R_PORT, &GPIO_InitStruct);
#endif

}

void MX_I2C_Init (void)
{
  I2C_InitTypeDef   I2C_InitStruct;
  
  /* I2C configuration */
  I2C_InitStruct.ClockSpeed       = 400000;
  I2C_InitStruct.DutyCycle        = I2C_DUTYCYCLE_16_9;
  I2C_InitStruct.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  I2C_InitStruct.DualAddressMode  = I2C_DUALADDRESS_DISABLED;
  I2C_InitStruct.NoStretchMode    = I2C_NOSTRETCH_DISABLED;
  I2C_InitStruct.GeneralCallMode  = I2C_GENERALCALL_DISABLED;
  I2C_InitStruct.OwnAddress1      = 0x0;
  I2C_InitStruct.OwnAddress2      = 0x0;
  
#ifdef IMU_L
  /* I2C peripheral initialization */
  IMU_L_I2C_Handler.Instance  = IMU_L_I2C;
  IMU_L_I2C_Handler.Init      = I2C_InitStruct;
  
  HAL_I2C_Init (&IMU_L_I2C_Handler);
#endif
    
#ifdef IMU_R
	/* I2C peripheral initialization */
  IMU_R_I2C_Handler.Instance  = IMU_R_I2C;
  IMU_R_I2C_Handler.Init      = I2C_InitStruct;
  
  HAL_I2C_Init (&IMU_R_I2C_Handler);
#endif
}


static void MX_IMU_Init (void)
{
#ifdef IMU_L
  /* IMU initialization */
  IMU_L_Handler.I2Cx                        = &IMU_L_I2C_Handler;
  IMU_L_Handler.Init.Clock_Source           = MPU9150_CLOCK_SRC_GYRO_X_AXIS;
  IMU_L_Handler.Init.LowPass_Filter         = IMU_LP_FILTER;
  IMU_L_Handler.Init.SampleRate_Divider     = IMU_SMPLRT(1000);
  IMU_L_Handler.Init.Accel_FullScale_Range  = MPU9150_ACCEL_FULLSCALE_2;
  IMU_L_Handler.Init.Gyro_FullScale_Range   = MPU9150_GYRO_FULLSCALE_2000;
  IMU_L_Handler.IRQ.Mode                    = MPU9150_INTERRUPT_MODE_PUSH_PULL;
  IMU_L_Handler.IRQ.Level                   = MPU9150_INTERRUPT_LEVEL_HIGH;
  IMU_L_Handler.IRQ.Latched                 = MPU9150_INTERRUPT_LATCHED;
  IMU_L_Handler.IRQ.Sources                 = MPU9150_FIFO_ACCEL  |
                                              MPU9150_FIFO_GYRO_X |
                                              MPU9150_FIFO_GYRO_Y |
                                              MPU9150_FIFO_GYRO_Z ;
  IMU_L_Handler.Click.MaxForce              = IMU_MAX_FORCE;
  IMU_L_Handler.Click.Threshold             = IMU_GYRO_THRESHOLD;
  
  MPU9150_Init (&IMU_L_Handler);
#endif
	
#ifdef IMU_R
  /* IMU initialization */
  IMU_R_Handler.I2Cx                        = &IMU_R_I2C_Handler;
  IMU_R_Handler.Init.Clock_Source           = MPU9150_CLOCK_SRC_GYRO_X_AXIS;
  IMU_R_Handler.Init.LowPass_Filter         = IMU_LP_FILTER;
  IMU_R_Handler.Init.SampleRate_Divider     = IMU_SMPLRT(1000);
  IMU_R_Handler.Init.Accel_FullScale_Range  = MPU9150_ACCEL_FULLSCALE_2;
  IMU_R_Handler.Init.Gyro_FullScale_Range   = MPU9150_GYRO_FULLSCALE_2000;
  IMU_R_Handler.IRQ.Mode                    = MPU9150_INTERRUPT_MODE_PUSH_PULL;
  IMU_R_Handler.IRQ.Level                   = MPU9150_INTERRUPT_LEVEL_HIGH;
  IMU_R_Handler.IRQ.Latched                 = MPU9150_INTERRUPT_LATCHED;
  IMU_R_Handler.IRQ.Sources                 = MPU9150_FIFO_ACCEL  |
                                              MPU9150_FIFO_GYRO_X |
                                              MPU9150_FIFO_GYRO_Y |
                                              MPU9150_FIFO_GYRO_Z ;
  IMU_R_Handler.Click.MaxForce              = IMU_MAX_FORCE;
  IMU_R_Handler.Click.Threshold             = IMU_GYRO_THRESHOLD;
  
  MPU9150_Init (&IMU_R_Handler);
#endif
}


static void MIDI_Thread(void const * argument)
{
	uint32_t prevWakeTime;
	
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
		
	prevWakeTime = osKernelSysTick ();

  for(;;)
  {
		MPU9150_ReadAccel   (&IMU_L_Handler);
		MPU9150_ReadGyro    (&IMU_L_Handler);
			
		MPU9150_DetectClick (&IMU_L_Handler);
			
		if(IMU_L_Handler.Click.Z.Clicked)
		{
			if (IMU_L_Handler.Buffer[MPU9150_BUFF_ACCEL_Y] > 0x2000)
			{
				MIDI_SendMsg(0x99, 0x26, IMU_L_Handler.Click.Z.Velocity);
			}
			else
			{
				MIDI_SendMsg(0x99, 0x31, IMU_L_Handler.Click.Z.Velocity);
			}
			
			HAL_GPIO_TogglePin(LED_MIDI_PORT, LED_MIDI_PIN);
			
			IMU_L_Handler.Click.Z.Clicked = 0;
		}
			
    if(IMU_R_Handler.Click.Z.Clicked)
		{
			MIDI_SendMsg(0x99, 0x2A, IMU_R_Handler.Click.Z.Velocity);
			HAL_GPIO_TogglePin(LED_MIDI_PORT, LED_MIDI_PIN);
			
			IMU_R_Handler.Click.Z.Clicked = 0;	
		}
    
    osDelayUntil(prevWakeTime, 2);
		prevWakeTime = osKernelSysTick ();
  }
}


/* Public functions definitions ----------------------------------------------*/

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
	HAL_GPIO_WritePin (LED_ERR_PORT, LED_ERR_PIN, GPIO_PIN_SET);
	
	__disable_irq ();
  while(1) ;
}


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
