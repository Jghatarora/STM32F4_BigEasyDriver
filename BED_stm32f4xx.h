#ifndef __BED_STMF4XX_H
#define __BED_STMF4XX_H
#include "stm32f4xx_hal_tim.h"              // Keil::Device:STM32Cube HAL:Common


extern TIM_handleTypeDef	e;
void TIM2_Init(void);

/* Functions */

void step(uint32_t steps);



#endif
