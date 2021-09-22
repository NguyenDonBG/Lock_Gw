#ifndef __DRIVER_UART_H
#define __DRIVER_UART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

void UART1_Init_A9A10(uint16_t baudrate);
int _write(int file, char *ptr, int len);
void USART1_IRQHandler();
void UART3_Config(uint32_t baudrate);
void UART_SendChar(USART_TypeDef *USARTx, char data);
void UART_PutStr(USART_TypeDef *USARTx, char *Str);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_UART_H */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
