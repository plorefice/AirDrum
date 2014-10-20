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
static uint8_t wr_buf[MPU9150_MAX_WRITE + 1]; // Space for address

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
	
	/* Set MPU9150 Clock Source and power on the device */
	ctrl = (uint8_t)(hmpu->Init.Clock_Source);
	MPU9150_Write (hmpu, MPU9150_PWR_MGMT_1_REG_ADDR, &ctrl, 1);
	
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
	                 hmpu->IRQ.Latched);
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
	/* Transmit buffer */
	MPU9150_I2C_WriteBuffer (hmpu->I2Cx,
	                         MPU9150_I2C_ADDR, 
	                         reg_addr, 
	                         buf,
                           len);
}


/**
  * @brief  Read the value of the accelerometer from the MPU9150.
	* @param  pBuffer: Pointer to a buffer of size int16_t[3].
  * @retval None
  */
void MPU9150_ReadAccel (MPU9150_HandleTypeDef  *hmpu, int16_t *buf)
{
	MPU9150_Read (hmpu, MPU9150_ACCEL_XOUT_H_REG_ADDR, (uint8_t *)buf, 6);

	buf[0] = (int16_t)(bswap16(buf[0]));
	buf[1] = (int16_t)(bswap16(buf[1]));
	buf[2] = (int16_t)(bswap16(buf[2]));
}


/**
  * @brief  Read the value of the gyroscope from the MPU9150.
	* @param  pBuffer: Pointer to a buffer of size int16_t[3].
  * @retval None
  */
void MPU9150_ReadGyro (MPU9150_HandleTypeDef  *hmpu, int16_t *buf)
{
	MPU9150_Read (hmpu, MPU9150_GYRO_XOUT_H_REG_ADDR, (uint8_t *)buf, 6);

	buf[0] = (int16_t)(bswap16(buf[0]));
	buf[1] = (int16_t)(bswap16(buf[1]));
	buf[2] = (int16_t)(bswap16(buf[2]));
}


/**
  * @brief  Read the value of the accelerations from the MPU9150.
	* @param  pBuffer: Pointer to a buffer of adequate size.
  * @retval Number of samples available in the FIFO.
  */
uint16_t MPU9150_ReadFIFO (MPU9150_HandleTypeDef  *hmpu, int16_t *buf)
{
	uint16_t nSamples;
	uint16_t i;
	
	/* Read number of samples */
	MPU9150_Read (hmpu, MPU9150_FIFO_COUNTH_REG_ADDR, (uint8_t *)&nSamples, 2);
	nSamples = (uint16_t)(bswap16(nSamples));

	/* Read data */
	for (i = 0; i < (nSamples >> 1); i++)
	{
		uint8_t *buff = (uint8_t *)&buf[i];
		
		MPU9150_Read (hmpu, MPU9150_FIFO_R_W_REG_ADDR, buff++, 1);
		MPU9150_Read (hmpu, MPU9150_FIFO_R_W_REG_ADDR, buff  , 1);
		
		buf[i] = (int16_t)(bswap16(buf[i]));
	}
	
	return nSamples;
}


void MPU9150_DetectClick (MPU9150_HandleTypeDef *hmpu, int16_t * buffer)
{
	int16_t x = buffer[0];
	int16_t y = buffer[1];
	int16_t z = buffer[2];
	
	/*
	if(th_x){
		if(x < TH) {
			
			if(max_x > MAX_FORCE)
				max_x = MAX_FORCE;
			IMU_ClickStruct->x.vel  = (max_x/MAX_FORCE) * 127;
			IMU_ClickStruct->x.click = 1;
			th_x = 0;
			max_x = 0;
		} else if(x > max_x) {
			max_x = x;
		}
	} else if(x > TH){
		max_x = x;
		th_x = 1;
	}
	
	if(th_y && y < TH){
		
		if(max_y > MAX_FORCE)
			max_y = MAX_FORCE;
		IMU_ClickStruct->y.vel = (max_y/MAX_FORCE) * 127;
		IMU_ClickStruct->y.click = 1;
		th_y = 0;
	}
	else if(y > TH){
		max_y = y;
		th_y = 1;
	}
	*/
	
	if(th_z)                                 // If I'm detecting a click
	{
		if (z >= th_z)                         // and I'm above the threshold
		{ 
			max_z = (z > max_z) ? z : max_z;     // update the maximum.
		}
		else {                                 // If I'm not above the threshold
			IMU_ClickStruct->z.click = 1;        // a click has been detected
			IMU_ClickStruct->z.vel = (uint8_t)   // and the velocity is calculated
			  ((max_z / MAX_FORCE) * 127.0f);    // based on the strength.
			th_z = 0;                            // Save everything and reset click
			max_z = 0;                           // and maximum strength.
		}
	}
	else if(z > TH)                          // If I have not started dectection,
	{
		max_z = z;                             // updated the maximum
		th_z = 1;                              // and start detecting.
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
	MPU9150_I2C_WriteBuffer (hmpu->I2Cx, MPU9150_I2C_ADDR, MPU9150_INT_PIN_CFG_REG_ADDR, &Data, 1);
	
	/* Read WHO_AM_I register of AK8975C */
	MPU9150_I2C_ReadBuffer (hmpu->I2Cx, AK8975C_I2C_ADDR, AK8975C_WIA_REG_ADDR, &WAI, 1);
	
	/* Disable direct access to AUX I2C bus */
	Data = 0x00;
	MPU9150_I2C_WriteBuffer (hmpu->I2Cx, MPU9150_I2C_ADDR, MPU9150_INT_PIN_CFG_REG_ADDR, &Data, 1);
	
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
	if (len == 0)
		return;	
	
	while (HAL_I2C_GetState (hi2c) == HAL_I2C_STATE_BUSY) ;
	
	HAL_I2C_Master_Transmit (hi2c, dev_addr, &reg_addr, 1, 1000);	
	HAL_I2C_Master_Receive  (hi2c, dev_addr, buf, len, 1000);
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
	if (len == 0 || len > MPU9150_MAX_WRITE)
		return;
	
	wr_buf[0] = reg_addr;
	memcpy (wr_buf + 1, buf, len);
	
	HAL_I2C_Master_Transmit (hi2c, dev_addr, wr_buf, len + 1, 1000);
}


/******************* (C) COPYRIGHT 2014 Pietro Lorefice ********END OF FILE****/
