#include "driver_uart.h"
char Rx_bufArr[256];
char uart1_rx[256];
static uint8_t rx_count = 0;
static uint8_t index   = 0;

void UART1_Init_A9A10(uint16_t baudrate)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);       // Configure the USART1
  USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);//
  NVIC_EnableIRQ(USART1_IRQn);//
  USART_Cmd(USART1, ENABLE);  // enable UART1

  {
    GPIO_InitTypeDef	gpio_init_struct;

    // GPIOA PIN9 alternative function Tx
    gpio_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_10MHz;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_init_struct);

    // GPIOA PIN10 alternative function Rx
    gpio_init_struct.GPIO_Pin = GPIO_Pin_10;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &gpio_init_struct);
  }

}

int _write(int file, char *ptr, int len)
{
  for (int i = len; i != 0; i--)
  {
    while ((USART1->SR & USART_FLAG_TXE) == 0);
    USART1->DR = *ptr++;
  }
  return len;
}

void UART3_Config(uint32_t baudrate)
{
  /*Cap clock cho USART và port su dung*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	USART_InitTypeDef			UART_InitStructure;
	GPIO_InitTypeDef			GPIO_InitStructure;
	/* Cau Tx mode AF_PP, Rx mode FLOATING  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*Cau hinh USART*/
	UART_InitStructure.USART_BaudRate = baudrate;
	UART_InitStructure.USART_WordLength = USART_WordLength_8b;
	UART_InitStructure.USART_StopBits = USART_StopBits_1;
	UART_InitStructure.USART_Parity = USART_Parity_No;
	UART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &UART_InitStructure);
	USART_ITConfig(USART3,USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART3_IRQn);
	/* Cho phep UART hoat dong */
	USART_Cmd(USART3, ENABLE);

}

 void UART_SendChar(USART_TypeDef *USARTx, char data){
    USARTx->DR = 0x00000000;
    USART_SendData(USARTx,data);
    //TxE = 1: Data is transferred to the shift register)
    //TxE = 0; Data is not transferred to the shift register
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
}
void UART_PutStr(USART_TypeDef *USARTx, char *Str){
    while(*Str){
        UART_SendChar(USARTx, *Str);
        Str++;
    }
}

 uint8_t USART_GetChar(USART_TypeDef* USARTx){
    uint8_t Data;
    while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET);
    Data = (uint8_t)USART_ReceiveData(USARTx);
    return Data;
}

void USART3_IRQHandler()// đọc dữ liệu từ UART gửi xuống và lưu vào mảng
{
  uint8_t chartoreceive = USART_GetChar(USART3);// gán biến bằng dữ liệu uart gửi xuống
  Rx_bufArr[rx_count] = chartoreceive;// lưu dữ liệu vào mảngb
  rx_count++;
  if(Rx_bufArr[rx_count-1] == '\n')
  {
    rx_count = 0;
  }
}

void USART1_IRQHandler()// đọc dữ liệu từ UART gửi xuống và lưu vào mảng
{
  uint8_t chartoreceive = USART_GetChar(USART1);// gán biến bằng dữ liệu uart gửi xuống
  uart1_rx[index] = chartoreceive;
  index++;
  if(uart1_rx[index-1] == '\n')
  {
      index = 0;
  }
}
