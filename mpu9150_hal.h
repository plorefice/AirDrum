/**
  ******************************************************************************
  * @file    mpu9150_hal.h
  * @author  Pietro Lorefice
  * @version V1.0.0
  * @date    18-October-2014
  * @brief   This file contains all the functions prototypes for the
	*          mpu9150_hal.c firmware driver.
  ******************************************************************************
  * @attention
  *
  * (C) COPYRIGHT 2014 Pietro Lorefice
  ******************************************************************************  
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPU9150_HAL_H
#define __MPU9150_HAL_H

#ifdef __cplusplus
extern "C" {
#endif
    
/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#include "stm32f4xx_hal.h"
	
/* Exported types ------------------------------------------------------------*/
	
/* MPU9150 Base config. */
typedef struct
{
	uint8_t  Clock_Source;
	uint8_t  SampleRate_Divider;
	uint8_t  LowPass_Filter;
	uint8_t  Gyro_FullScale_Range;
	uint8_t  Accel_FullScale_Range;
} MPU9150_InitTypeDef;
	
/* MPU9150 Interrupt config. */
typedef struct
{
	uint8_t Level;
	uint8_t Mode;
	uint8_t Latched;
	uint8_t Sources;
} MPU9150_IRQTypeDef;

/* MPU9150 Handle */
typedef struct 
{
	I2C_HandleTypeDef   *I2Cx;
	MPU9150_InitTypeDef  Init;
	MPU9150_IRQTypeDef   IRQ;
} MPU9150_HandleTypeDef;


/* Exported macros -----------------------------------------------------------*/

#define bswap16(X)                (((X & 0xFF00) >> 8) | ((X & 0x00FF) << 8))

/* Exported constants --------------------------------------------------------*/

/* Max write length (in bytes) */
#define MPU9150_MAX_WRITE                                          0x10

/* I2C Addresses */
#define MPU9150_I2C_ADDR                                         ((0x69)<<1)
#define AK8975C_I2C_ADDR                                         ((0x0C)<<1)

/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/

#define MPU9150_SELF_TEST_X_REG_ADDR                               0x0D
#define MPU9150_SELF_TEST_Y_REG_ADDR                               0x0E
#define MPU9150_SELF_TEST_Z_REG_ADDR                               0x0F
#define MPU9150_SELF_TEST_A_REG_ADDR                               0x10
#define MPU9150_SMPLRT_DIV_REG_ADDR                                0x19
#define MPU9150_CONFIG_REG_ADDR                                    0x1A
#define MPU9150_GYRO_CONFIG_REG_ADDR                               0x1B
#define MPU9150_ACCEL_CONFIG_REG_ADDR                              0x1C
#define MPU9150_FIFO_EN_REG_ADDR                                   0x23
#define MPU9150_I2C_MST_CTRL_REG_ADDR                              0x24
#define MPU9150_I2C_SLV0_ADDR_REG_ADDR                             0x25
#define MPU9150_I2C_SLV0_REG_REG_ADDR                              0x26
#define MPU9150_I2C_SLV0_CTRL_REG_ADDR                             0x27
#define MPU9150_I2C_SLV1_ADDR_REG_ADDR                             0x28
#define MPU9150_I2C_SLV1_REG_REG_ADDR                              0x29
#define MPU9150_I2C_SLV1_CTRL_REG_ADDR                             0x2A
#define MPU9150_I2C_SLV2_ADDR_REG_ADDR                             0x2B
#define MPU9150_I2C_SLV2_REG_REG_ADDR                              0x2C
#define MPU9150_I2C_SLV2_CTRL_REG_ADDR                             0x2D
#define MPU9150_I2C_SLV3_ADDR_REG_ADDR                             0x2E
#define MPU9150_I2C_SLV3_REG_REG_ADDR                              0x2F
#define MPU9150_I2C_SLV3_CTRL_REG_ADDR                             0x30
#define MPU9150_I2C_SLV4_ADDR_REG_ADDR                             0x31
#define MPU9150_I2C_SLV4_REG_REG_ADDR                              0x32
#define MPU9150_I2C_SLV4_DO_REG_ADDR                               0x33
#define MPU9150_I2C_SLV4_CTRL_REG_ADDR                             0x34
#define MPU9150_I2C_SLV4_DI_REG_ADDR                               0x35
#define MPU9150_I2C_MST_STATUS_REG_ADDR                            0x36
#define MPU9150_INT_PIN_CFG_REG_ADDR                               0x37
#define MPU9150_INT_ENABLE_REG_ADDR                                0x38
#define MPU9150_INT_STATUS_REG_ADDR                                0x3A
#define MPU9150_ACCEL_XOUT_H_REG_ADDR                              0x3B
#define MPU9150_ACCEL_XOUT_L_REG_ADDR                              0x3C
#define MPU9150_ACCEL_YOUT_H_REG_ADDR                              0x3D
#define MPU9150_ACCEL_YOUT_L_REG_ADDR                              0x3E
#define MPU9150_ACCEL_ZOUT_H_REG_ADDR                              0x3F
#define MPU9150_ACCEL_ZOUT_L_REG_ADDR                              0x40
#define MPU9150_TEMP_OUT_H_REG_ADDR                                0x41
#define MPU9150_TEMP_OUT_L_REG_ADDR                                0x42
#define MPU9150_GYRO_XOUT_H_REG_ADDR                               0x43
#define MPU9150_GYRO_XOUT_L_REG_ADDR                               0x44
#define MPU9150_GYRO_YOUT_H_REG_ADDR                               0x45
#define MPU9150_GYRO_YOUT_L_REG_ADDR                               0x46
#define MPU9150_GYRO_ZOUT_H_REG_ADDR                               0x47
#define MPU9150_GYRO_ZOUT_L_REG_ADDR                               0x48
#define MPU9150_EXT_SENS_DATA_00_REG_ADDR                          0x49
#define MPU9150_EXT_SENS_DATA_01_REG_ADDR                          0x4A
#define MPU9150_EXT_SENS_DATA_02_REG_ADDR                          0x4B
#define MPU9150_EXT_SENS_DATA_03_REG_ADDR                          0x4C
#define MPU9150_EXT_SENS_DATA_04_REG_ADDR                          0x4D
#define MPU9150_EXT_SENS_DATA_05_REG_ADDR                          0x4E
#define MPU9150_EXT_SENS_DATA_06_REG_ADDR                          0x4F
#define MPU9150_EXT_SENS_DATA_07_REG_ADDR                          0x50
#define MPU9150_EXT_SENS_DATA_08_REG_ADDR                          0x51
#define MPU9150_EXT_SENS_DATA_09_REG_ADDR                          0x52
#define MPU9150_EXT_SENS_DATA_10_REG_ADDR                          0x53
#define MPU9150_EXT_SENS_DATA_11_REG_ADDR                          0x54
#define MPU9150_EXT_SENS_DATA_12_REG_ADDR                          0x55
#define MPU9150_EXT_SENS_DATA_13_REG_ADDR                          0x56
#define MPU9150_EXT_SENS_DATA_14_REG_ADDR                          0x57
#define MPU9150_EXT_SENS_DATA_15_REG_ADDR                          0x58
#define MPU9150_EXT_SENS_DATA_16_REG_ADDR                          0x59
#define MPU9150_EXT_SENS_DATA_17_REG_ADDR                          0x5A
#define MPU9150_EXT_SENS_DATA_18_REG_ADDR                          0x5B
#define MPU9150_EXT_SENS_DATA_19_REG_ADDR                          0x5C
#define MPU9150_EXT_SENS_DATA_20_REG_ADDR                          0x5D
#define MPU9150_EXT_SENS_DATA_21_REG_ADDR                          0x5E
#define MPU9150_EXT_SENS_DATA_22_REG_ADDR                          0x5F
#define MPU9150_EXT_SENS_DATA_23_REG_ADDR                          0x60
#define MPU9150_I2C_SLV0_DO_REG_ADDR                               0x63
#define MPU9150_I2C_SLV1_DO_REG_ADDR                               0x64
#define MPU9150_I2C_SLV2_DO_REG_ADDR                               0x65
#define MPU9150_I2C_SLV3_DO_REG_ADDR                               0x66
#define MPU9150_I2C_MST_DELAY_CTRL_REG_ADDR                        0x67
#define MPU9150_SIGNAL_PATH_RESET_REG_ADDR                         0x68
#define MPU9150_USER_CTRL_REG_ADDR                                 0x6A
#define MPU9150_PWR_MGMT_1_REG_ADDR                                0x6B
#define MPU9150_PWR_MGMT_2_REG_ADDR                                0x6C
#define MPU9150_FIFO_COUNTH_REG_ADDR                               0x72
#define MPU9150_FIFO_COUNTL_REG_ADDR                               0x73
#define MPU9150_FIFO_R_W_REG_ADDR                                  0x74
#define MPU9150_WHO_AM_I_REG_ADDR                                  0x75

#define AK8975C_WIA_REG_ADDR                                       0x00
#define AK8975C_INFO_REG_ADDR                                      0x01
#define AK8975C_ST1_REG_ADDR                                       0x02
#define AK8975C_HXL_REG_ADDR                                       0x03
#define AK8975C_HXH_REG_ADDR                                       0x04
#define AK8975C_HYL_REG_ADDR                                       0x05
#define AK8975C_HYH_REG_ADDR                                       0x06
#define AK8975C_HZL_REG_ADDR                                       0x07
#define AK8975C_HZH_REG_ADDR                                       0x08
#define AK8975C_ST2_REG_ADDR                                       0x09
#define AK8975C_CNTL_REG_ADDR                                      0x0A
#define AK8975C_RSV_REG_ADDR                                       0x0B
#define AK8975C_ASTC_REG_ADDR                                      0x0C
#define AK8975C_TS1_REG_ADDR                                       0x0D
#define AK8975C_TS2_REG_ADDR                                       0x0E
#define AK8975C_I2CDIS_REG_ADDR                                    0x0F
#define AK8975C_ASAX_REG_ADDR                                      0x10
#define AK8975C_ASAY_REG_ADDR                                      0x11
#define AK8975C_ASAZ_REG_ADDR                                      0x12

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

/* Accelerometer sensitivity */
#define MPU9150_ACCEL_SENSITIVITY_2                       16384.0f /*  LSB/mg */
#define MPU9150_ACCEL_SENSITIVITY_4                       8192.0f  /*  LSB/mg */
#define MPU9150_ACCEL_SENSITIVITY_8                       4096.0f  /*  LSB/mg */
#define MPU9150_ACCEL_SENSITIVITY_16                      2048.0f  /*  LSB/mg */

/* Gyroscope sensitivity */
#define MPU9150_GYRO_SENSITIVITY_250                      131.0f   /*  LSB/°/s */
#define MPU9150_GYRO_SENSITIVITY_500                      65.5f    /*  LSB/°/s */
#define MPU9150_GYRO_SENSITIVITY_1000                     32.8f    /*  LSB/°/s */
#define MPU9150_GYRO_SENSITIVITY_2000                     16.4f    /*  LSB/°/s */

/* External synchronization set */
#define MPU9150_EXT_SYNC_SET_0                            ((uint8_t)0x00)
#define MPU9150_EXT_SYNC_SET_1                            ((uint8_t)0x08)
#define MPU9150_EXT_SYNC_SET_2                            ((uint8_t)0x10)
#define MPU9150_EXT_SYNC_SET_3                            ((uint8_t)0x18)
#define MPU9150_EXT_SYNC_SET_4                            ((uint8_t)0x20)
#define MPU9150_EXT_SYNC_SET_5                            ((uint8_t)0x28)
#define MPU9150_EXT_SYNC_SET_6                            ((uint8_t)0x30)
#define MPU9150_EXT_SYNC_SET_7                            ((uint8_t)0x38)

/* Digital LP Filter configuration */
#define MPU9150_LOWPASSFILTER_0                           ((uint8_t)0x00)
#define MPU9150_LOWPASSFILTER_1                           ((uint8_t)0x01)
#define MPU9150_LOWPASSFILTER_2                           ((uint8_t)0x02)
#define MPU9150_LOWPASSFILTER_3                           ((uint8_t)0x03)
#define MPU9150_LOWPASSFILTER_4                           ((uint8_t)0x04)
#define MPU9150_LOWPASSFILTER_5                           ((uint8_t)0x05)
#define MPU9150_LOWPASSFILTER_6                           ((uint8_t)0x06)

/* Gyroscope Fullscale selection */
#define MPU9150_GYRO_FULLSCALE_250                        ((uint8_t)0x00)
#define MPU9150_GYRO_FULLSCALE_500                        ((uint8_t)0x08)
#define MPU9150_GYRO_FULLSCALE_1000                       ((uint8_t)0x10)
#define MPU9150_GYRO_FULLSCALE_2000                       ((uint8_t)0x18)

/* Accelerometer Fullscale selection */
#define MPU9150_ACCEL_FULLSCALE_2                         ((uint8_t)0x00)
#define MPU9150_ACCEL_FULLSCALE_4                         ((uint8_t)0x08)
#define MPU9150_ACCEL_FULLSCALE_8                         ((uint8_t)0x10)
#define MPU9150_ACCEL_FULLSCALE_16                        ((uint8_t)0x18)

/* FIFO data selection */
#define MPU9150_FIFO_GYRO_X                               ((uint8_t)0x40)
#define MPU9150_FIFO_GYRO_Y                               ((uint8_t)0x20)
#define MPU9150_FIFO_GYRO_Z                               ((uint8_t)0x10)
#define MPU9150_FIFO_ACCEL                                ((uint8_t)0x08)

/* IRQ pin configuration */
#define MPU9150_INTERRUPT_LEVEL_HIGH                      ((uint8_t)0x00)
#define MPU9150_INTERRUPT_LEVEL_LOW                       ((uint8_t)0x80)
#define MPU9150_INTERRUPT_MODE_PUSH_PULL                  ((uint8_t)0x00)
#define MPU9150_INTERRUPT_MODE_OPEN_DRAIN                 ((uint8_t)0x40)
#define MPU9150_INTERRUPT_PULSE                           ((uint8_t)0x00)
#define MPU9150_INTERRUPT_LATCHED                         ((uint8_t)0x20)

/* IRQ source selection */
#define MPU9150_INTERRUPT_SOURCE_NONE                     ((uint8_t)0x00)
#define MPU9150_INTERRUPT_SOURCE_FIFO_OF                  ((uint8_t)0x10)
#define MPU9150_INTERRUPT_SOURCE_DATA_RDY                 ((uint8_t)0x01)

/* Clock source selection */
#define MPU9150_CLOCK_SRC_INTERNAL_OSC                    ((uint8_t)0x00)
#define MPU9150_CLOCK_SRC_GYRO_X_AXIS                     ((uint8_t)0x01)
#define MPU9150_CLOCK_SRC_GYRO_Y_AXIS                     ((uint8_t)0x02)
#define MPU9150_CLOCK_SRC_GYRO_Z_AXIS                     ((uint8_t)0x03)
#define MPU9150_CLOCK_SRC_EXT_32_7_KHZ                    ((uint8_t)0x04)
#define MPU9150_CLOCK_SRC_EXT_19_2_MHZ                    ((uint8_t)0x05)
#define MPU9150_CLOCK_SRC_RESET                           ((uint8_t)0x07)


/* Exported functions --------------------------------------------------------*/
	
void     MPU9150_Init  (MPU9150_HandleTypeDef *hmpu);
void     MPU9150_Read  (MPU9150_HandleTypeDef *hmpu, uint8_t reg_addr, uint8_t *buf, uint8_t len);
void     MPU9150_Write (MPU9150_HandleTypeDef *hmpu, uint8_t reg_addr, uint8_t *buf, uint8_t len);

void     MPU9150_ReadAccel (MPU9150_HandleTypeDef *hmpu, int16_t *pBuffer);
void     MPU9150_ReadGyro  (MPU9150_HandleTypeDef *hmpu, int16_t *pBuffer);
uint16_t MPU9150_ReadFIFO  (MPU9150_HandleTypeDef *hmpu, int16_t *pBuffer);


#ifdef __cplusplus
}
#endif

#endif        /* __MPU9150_HAL_H */


/******************* (C) COPYRIGHT 2011 Pietro Lorefice ********END OF FILE****/
