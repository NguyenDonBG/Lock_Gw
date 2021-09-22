/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


void SysTick_Init();
void SysTick_Handler();
uint64_t SysTick64();
uint32_t SysTick32();
uint32_t SysTick24();
uint64_t SysTick_Millis();
uint64_t SysTick_Micros();
void delay_us(unsigned long us);
void delay_ms(unsigned long ms);


#ifdef __cplusplus
}
#endif

#endif /* __delay_H */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2020 Giaosu*****END OF FILE****/
