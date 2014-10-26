#ifndef __STM32F4XX_EMPL_DRV_H
#define __STM32F4XX_EMPL_DRV_H

#include "stm32f4xx_hal.h"

int stm32f4xx_i2c_write (unsigned char slave_addr,
                         unsigned char reg_addr, 
                         unsigned char length, 
                         unsigned char const *data);
													
int stm32f4xx_i2c_read  (unsigned char slave_addr,
                         unsigned char reg_addr, 
                         unsigned char length, 
                         unsigned char *data);

void stm32f4xx_delay_ms (unsigned long num_ms);
void stm32f4xx_get_ms   (unsigned long *count);

#endif /* __STM32F4XX_EMPL_DRV_H */
