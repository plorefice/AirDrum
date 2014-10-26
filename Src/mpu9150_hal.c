/**
  ******************************************************************************
  * @file    mpu9150_hal.c
  * @author  Pietro Lorefice
  * @version V1.0.0
  * @date    18-October-2014
  * @brief   This file provides a set of functions needed to manage the MPU9150
  *          6DoF MEMS IMU.
  ******************************************************************************
  * @attention
  *
  * (C) COPYRIGHT 2014 Pietro Lorefice
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mpu9150_hal.h"

#include <string.h>
    
/* Private variables ---------------------------------------------------------*/
/* Private Function Prototypes -----------------------------------------------*/
static void MPU9150_I2C_ReadBuffer  (I2C_HandleTypeDef  *hi2c,
                                     uint8_t             dev_addr,
                                     uint8_t             reg_addr,
                                     uint8_t            *buf,
                                     uint8_t             len);

static void MPU9150_I2C_WriteBuffer (I2C_HandleTypeDef  *hi2c,
                                     uint8_t             dev_addr,
                                     uint8_t             reg_addr,
                                     uint8_t            *buf,
                                     uint8_t             len);

static void MPU9150_Compass_Init    (MPU9150_HandleTypeDef *hmpu);

extern void Error_Handler (void);

/* Public Function Definitions -----------------------------------------------*/

/**
  * @brief  Initializes the MPU9150.
  * @param  MPU9150_InitStruct: pointer to a MPU9150_InitTypeDef structure 
  *         that contains the configuration setting for the MPU9150.
  * @retval None
  */
void MPU9150_Init (MPU9150_HandleTypeDef *hmpu)
{
  uint8_t ctrl = 0x00;
  
  /* Reset the device */
  ctrl = 0x80;
  MPU9150_Write (hmpu, MPU9150_PWR_MGMT_1_REG_ADDR, &ctrl, 1);
  
  HAL_Delay(50);
  
  /* Set MPU9150 Clock Source and power on the device */
  ctrl = (uint8_t)(hmpu->Init.Clock_Source);
  MPU9150_Write (hmpu, MPU9150_PWR_MGMT_1_REG_ADDR, &ctrl, 1);
  
  HAL_Delay(50);
  
  /* Configure the Sample Rate Divider */
  ctrl = (uint8_t)(hmpu->Init.SampleRate_Divider);
  MPU9150_Write (hmpu, MPU9150_SMPLRT_DIV_REG_ADDR, &ctrl, 1);
  
  /* Configure the Digital LP Filter */
  ctrl = (uint8_t)(hmpu->Init.LowPass_Filter);
  MPU9150_Write (hmpu, MPU9150_CONFIG_REG_ADDR, &ctrl, 1);
  
  /* Configure the Gyroscope */
  ctrl = (uint8_t)(hmpu->Init.Gyro_FullScale_Range);
  MPU9150_Write (hmpu, MPU9150_GYRO_CONFIG_REG_ADDR, &ctrl, 1);

  /* Configure the Accelerometer */
  ctrl = (uint8_t)(hmpu->Init.Accel_FullScale_Range);
  MPU9150_Write (hmpu, MPU9150_ACCEL_CONFIG_REG_ADDR, &ctrl, 1);

  /* Configure FIFO sources */
  ctrl = (uint8_t)(hmpu->IRQ.Sources);
  MPU9150_Write (hmpu, MPU9150_FIFO_EN_REG_ADDR, &ctrl, 1);
  
  /* Reset and enable FIFO buffer */
  ctrl = 0x04;
  MPU9150_Write (hmpu, MPU9150_USER_CTRL_REG_ADDR, &ctrl, 1);
  
  ctrl = 0x40;
  MPU9150_Write (hmpu, MPU9150_USER_CTRL_REG_ADDR, &ctrl, 1);
  
  /* Enable interrupts */
  ctrl = (uint8_t)(hmpu->IRQ.Level   |
                   hmpu->IRQ.Mode    |
                   hmpu->IRQ.Latched |
                   0x10              );
  MPU9150_Write (hmpu, MPU9150_INT_PIN_CFG_REG_ADDR, &ctrl, 1);
  
  /* Enable DRDY interrupt */
  ctrl = 0x01;
  MPU9150_Write (hmpu, MPU9150_INT_ENABLE_REG_ADDR, &ctrl, 1);
  
  /* Configure the Magnetometer */
  MPU9150_Compass_Init (hmpu);
}


/**
  * @brief  Writes one byte to the MPU9150.
  * @param  WriteAddr : MPU9150's internal address to write to.
  * @param  Data: Byte to write.
  * @retval None
  */
void MPU9150_Read (MPU9150_HandleTypeDef  *hmpu,
                   uint8_t                 reg_addr,
                   uint8_t                *buf,
                   uint8_t                 len)
{
  if (len == 0)
    return;	
  
  /* Receive buffer */
  MPU9150_I2C_ReadBuffer (hmpu->I2Cx,
                          MPU9150_I2C_ADDR, 
                          reg_addr, 
                          buf,
                          len);
}


/**
  * @brief  Writes one byte to the MPU9150.
  * @param  WriteAddr : MPU9150's internal address to write to.
  * @param  pData: Pointer to the buffer to write.
  * @retval None
  */
void MPU9150_Write (MPU9150_HandleTypeDef  *hmpu,
                    uint8_t                 reg_addr,
                    uint8_t                *buf,
                    uint8_t                 len)
{
  uint8_t *tmp = (uint8_t *)(&(hmpu->Buffer[0]));

  if (len == 0 || len > MPU9150_MAX_XFER_SIZE)
    return;

  tmp[0] = reg_addr;
  memcpy (&tmp[1], &buf[0], len);

  /* Transmit buffer */
  MPU9150_I2C_WriteBuffer (hmpu->I2Cx,
                           MPU9150_I2C_ADDR, 
                           reg_addr, 
                           tmp,
                           len + 1);
}


void AK8975C_Read (MPU9150_HandleTypeDef  *hmpu,
                   uint8_t                 reg_addr,
                   uint8_t                *buf,
                   uint8_t                 len)
{
  if (len == 0)
    return;	

  /* Receive buffer */
  MPU9150_I2C_ReadBuffer (hmpu->I2Cx,
                          AK8975C_I2C_ADDR, 
                          reg_addr, 
                          buf,
                          len);
}


/**
  * @brief  Read the value of the accelerometer from the MPU9150.
	* @param  pBuffer: Pointer to a buffer of size int16_t[3].
  * @retval None
  */
void MPU9150_ReadAccel (MPU9150_HandleTypeDef *hmpu)
{
  MPU9150_Read (hmpu, MPU9150_ACCEL_XOUT_H_REG_ADDR,
               (uint8_t *)(&hmpu->Buffer[MPU9150_BUFF_ACCEL_X]), 6);

  hmpu->Buffer[MPU9150_BUFF_ACCEL_X] = (int16_t)(bswap16(hmpu->Buffer[MPU9150_BUFF_ACCEL_X]));
  hmpu->Buffer[MPU9150_BUFF_ACCEL_Y] = (int16_t)(bswap16(hmpu->Buffer[MPU9150_BUFF_ACCEL_Y]));
  hmpu->Buffer[MPU9150_BUFF_ACCEL_Z] = (int16_t)(bswap16(hmpu->Buffer[MPU9150_BUFF_ACCEL_Z]));
}


/**
  * @brief  Read the value of the gyroscope from the MPU9150.
	* @param  pBuffer: Pointer to a buffer of size int16_t[3].
  * @retval None
  */
void MPU9150_ReadGyro (MPU9150_HandleTypeDef *hmpu)
{
  MPU9150_Read (hmpu, MPU9150_GYRO_XOUT_H_REG_ADDR,
	             (uint8_t *)(&hmpu->Buffer[MPU9150_BUFF_GYRO_X]), 6);

  hmpu->Buffer[MPU9150_BUFF_GYRO_X] = (int16_t)(bswap16(hmpu->Buffer[MPU9150_BUFF_GYRO_X]));
  hmpu->Buffer[MPU9150_BUFF_GYRO_Y] = (int16_t)(bswap16(hmpu->Buffer[MPU9150_BUFF_GYRO_Y]));
  hmpu->Buffer[MPU9150_BUFF_GYRO_Z] = (int16_t)(bswap16(hmpu->Buffer[MPU9150_BUFF_GYRO_Z]));
}


/**
  * @brief  Read the value of the accelerations from the MPU9150.
	* @param  pBuffer: Pointer to a buffer of adequate size.
  * @retval Number of samples available in the FIFO.
  */
uint16_t MPU9150_ReadFIFO (MPU9150_HandleTypeDef *hmpu)
{
  uint16_t nSamples;
  uint16_t i;

  /* Read number of samples */
  MPU9150_Read (hmpu, MPU9150_FIFO_COUNTH_REG_ADDR, (uint8_t *)&nSamples, 2);
  nSamples = (uint16_t)(bswap16(nSamples));

  if (nSamples > MPU9150_MAX_XFER_SIZE)
  {
    Error_Handler();
  }

  /* Read data */
  for (i = 0; i < (nSamples >> 1); i++)
  {
    uint8_t *buff = (uint8_t *)(&(hmpu->Buffer[i]));

    MPU9150_Read (hmpu, MPU9150_FIFO_R_W_REG_ADDR, buff++, 1);
    MPU9150_Read (hmpu, MPU9150_FIFO_R_W_REG_ADDR, buff  , 1);

    hmpu->Buffer[i] = (int16_t)(bswap16(hmpu->Buffer[i]));
  }

  return nSamples;
}


void MPU9150_DetectClick (MPU9150_HandleTypeDef *hmpu)
{
  MPU9150_ClickTypeDef *cs = &(hmpu->Click);
  int16_t                z = hmpu->Buffer[MPU9150_BUFF_GYRO_Z];
  
  if(cs->Z.Clicking)                              // If I'm detecting a click
  {
    if (z >= cs->Threshold)                       // and I'm above the threshold
    { 
      cs->Z.Force = (z > cs->Z.Force) ?           // update the maximum.
                     z : cs->Z.Force;   
    }
    else                                          // If I'm not above the threshold
    {                                
      cs->Z.Clicked  = 1;                         // a click has been detected
      cs->Z.Velocity = (uint8_t)                  // and the velocity is calculated
        ((cs->Z.Force / (float)(cs->MaxForce)) * 127.0f);  // based on the strength.
      
      cs->Z.Clicking = 0;                         // Save everything and reset click
      cs->Z.Force    = 0;                         // and maximum strength.
    }
  }
  else if(z > cs->Threshold)                      // If I have not started dectection,
  {
    cs->Z.Force    = z;                           // updated the maximum
    cs->Z.Clicking = 1;                           // and start detecting.
  }
}


/* Private Function Definitions ----------------------------------------------*/

/**
  * @brief  Test the AK8975C by reading the WHO_AM_I register.
  * @retval 0 if successful, 1 otherwise.
  */
static uint8_t MPU9150_Compass_Test (MPU9150_HandleTypeDef *hmpu)
{
  uint8_t Data = 0x00;
  uint8_t WAI;

  /* Enable direct access to AUX I2C bus inside MPU9150 from the Discovery */
  Data = 0x02;
  MPU9150_Write (hmpu, MPU9150_INT_PIN_CFG_REG_ADDR, &Data, 1);

  /* Read WHO_AM_I register of AK8975C */
  AK8975C_Read  (hmpu, AK8975C_WIA_REG_ADDR, &WAI, 1);

  /* Disable direct access to AUX I2C bus */
  Data = 0x00;
  MPU9150_Write (hmpu, MPU9150_INT_PIN_CFG_REG_ADDR, &Data, 1);

  /* Wrong DevID */
  if (WAI != 0x48) return 1;

  return 0;
}


/**
  * @brief  Initialize the AK8975C magnetometer integrated in the MPU9150.
  * @retval None
  */
static void MPU9150_Compass_Init (MPU9150_HandleTypeDef *hmpu)
{
  /* Perform preliminary compass test */
  if (0x0 != MPU9150_Compass_Test (hmpu))
  {
		Error_Handler ();
  }
}


/**
  * @brief  Read multiple bytes from the MPU9150.
  * @param  I2Cx : I2C interface to which the MPU9150 is connected.
  * @param  DevAddress : I2C address of the device from which data must be read.
  * @param  RegAddress : Address of the first egister from which data must be read.
  * @param  pBuffer : Pointer to the buffer in which data will be stored.
  * @param  nBytes : Number of bytes to read.
  * @retval None
  */
static void MPU9150_I2C_ReadBuffer (I2C_HandleTypeDef  *hi2c,
                                    uint8_t             dev_addr,
                                    uint8_t             reg_addr,
                                    uint8_t            *buf,
                                    uint8_t             len)
{
  while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET) ;

  HAL_I2C_Master_Transmit (hi2c, dev_addr, &reg_addr, 1, 5);
  HAL_I2C_Master_Receive  (hi2c, dev_addr, buf, len, 5);
}


/**
  * @brief  Write multiple bytes to the MPU9150.
  * @param  I2Cx : I2C interface to which the MPU9150 is connected.
  * @param  DevAddress : I2C address of the device to which data must be written.
  * @param  RegAddress : Address of the register in which data must be written.
  * @param  pData : Pointer to the data to write.
  * @param  nBytes : Number of bytes to write.
  * @retval None
  */
static void MPU9150_I2C_WriteBuffer (I2C_HandleTypeDef  *hi2c,
                                     uint8_t             dev_addr,
                                     uint8_t             reg_addr,
                                     uint8_t            *buf,
                                     uint8_t             len)
{
  while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET) ;

  HAL_I2C_Master_Transmit (hi2c, dev_addr, buf, len, 5);
}


/******************* (C) COPYRIGHT 2014 Pietro Lorefice ********END OF FILE****/
