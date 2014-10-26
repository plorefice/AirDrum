#include "stm32f4xx_empl_drv.h"

int stm32f4xx_i2c_write (unsigned char slave_addr,
                         unsigned char reg_addr, 
                         unsigned char length, 
                         unsigned char const *data)
{
	return 1;
}

													
int stm32f4xx_i2c_read  (unsigned char slave_addr,
                         unsigned char reg_addr, 
                         unsigned char length, 
                         unsigned char *data)
{
	return 1;
}


void stm32f4xx_delay_ms  (unsigned long num_ms)
{
	
}


void stm32f4xx_get_ms    (unsigned long *count)
{
	
}
