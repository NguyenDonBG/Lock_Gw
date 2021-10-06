/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f10x_conf.h"
#include "driver_uart.h"
#include "delay.h"
#include <stdbool.h>
#include <string.h>

#define BUTTON_PAIR         GPIO_Pin_0
#define LED_OFF()           GPIO_SetBits(GPIOA, GPIO_Pin_6)
#define LED_ON()            GPIO_ResetBits(GPIOA, GPIO_Pin_6)

#define RF_CS_PIN                 GPIO_Pin_7
#define RF_SET_PIN                GPIO_Pin_8

#define STM32_UUID          ((uint32_t *)0x1FFFF7E8)


bool status_pair = false;
volatile uint8_t slot_rx_time = 0;
volatile uint8_t sync_send_time = 0;
volatile bool sync_status = 0;
volatile bool tx1_status = 0;
volatile bool tx2_status = 0;
volatile bool ping_status = 0;

extern char Rx_bufArr[64];
extern char uart1_rx[64];

typedef struct{
    char type[5];
    char UID[18];
    char id_node[4];
    char data[4];
}packet;


void Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef   Timer_InitStructure;
    Timer_InitStructure.TIM_Prescaler   = 7200 - 1;
    Timer_InitStructure.TIM_Period      = 10000 - 1;
    Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &Timer_InitStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    NVIC_InitTypeDef        NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel    = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd  = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM2, TIM_IT_Update) != RESET)
    {
        slot_rx_time++;
        sync_send_time++;
        if(slot_rx_time == 1) tx1_status = false;
        if(slot_rx_time == 3) tx2_status = false;
        if(slot_rx_time == 5) ping_status = false;;
        if(slot_rx_time > 10)
        {
            sync_status = false;
            slot_rx_time = 0;

        }
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    }
}
/**
  * @brief  Gpio_Init config gpio
  * @param
  * @retval
  */
void Gpio_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef   GPIO_Init_Structure;
    GPIO_Init_Structure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init_Structure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_Init_Structure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_Init_Structure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init_Structure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_Init_Structure);

    GPIO_Init_Structure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init_Structure.GPIO_Pin   = GPIO_Pin_6|RF_CS_PIN|RF_SET_PIN;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_Init_Structure);
}
/**
  * @brief  Reads the specified GPIO output data port.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @retval GPIO output data port value.
  */
void Button_Detect_Event(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool *status)
{
    bool status_button;
    if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 1)
    {
        status_button = true;
    }

    else if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0 && status_button == true)
    {
        status_button = false;
        *status = true;
    }
}
/**
  * @brief
  * @param
  * @retval
  */
bool Process_Message(char *str, char *type, char *ID_get_way, char *ID_node, char *mess)
{
    char *p_rx;
    char *p_type;
    char *p_id_getway;
    char *p_id_node;
    char *p_mess;
    if((str != NULL) && (type != NULL) && (ID_get_way != NULL) && (ID_node != NULL) && (mess != NULL))
    {
        p_rx = str;
        p_type = p_rx;
        p_rx = strchr(p_type,',');
        if(p_rx != NULL)
        {
            *p_rx = 0;
            strcpy(type,p_type);
        }
        p_id_getway = p_rx+1;
        p_rx = strchr(p_id_getway,',');
        if(p_rx != NULL)
        {
            *p_rx = 0;
            strcpy(ID_get_way,p_id_getway);
        }
        p_id_node = p_rx+1;
        p_rx = strchr(p_id_node,',');
        if(p_rx != NULL)
        {
            *p_rx = 0;
            strcpy(ID_node,p_id_node);
        }
        p_mess = p_rx+1;
        p_rx = strchr(p_mess, ',');
        if(p_rx != NULL)
        {
            *p_rx = 0;
            strcpy(mess, p_mess);
        }
        return true;
    }

    else{
        return false;
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void Send_Packet(packet *data)
{
    printf("%s,%s,%s,%s,\n",data->type, data->UID, data->id_node,data->data);
}
/**
  * @brief
  * @param
  * @retval
  */
void Task_Pair_Connect(void)
{
    LED_ON();
    packet data;
    Button_Detect_Event(GPIOA, BUTTON_PAIR, &status_pair);
    while(status_pair == true)
    {
        strcpy(data.type, "pair");
        sprintf(data.UID, "%x", STM32_UUID[0]&0xFFFFFF);
        strcpy(data.id_node, "TX1");
        strcpy(data.data, "pair");
        Send_Packet(&data);
        status_pair = false;
    }
    LED_OFF();
}

int main(void)
{
    SysTick_Init();
    Timer_Init();
    Gpio_Init();

    UART1_Init_A9A10(9600);
    UART3_Config(9600);
    LED_OFF();
    GPIO_SetBits(GPIOA, RF_SET_PIN);
    GPIO_ResetBits(GPIOA, RF_CS_PIN);
    while(1)
    {
        Task_Pair_Connect();
        if(slot_rx_time == 10 && sync_status == false)
        {
            printf("sync\n");
            sync_status = true;
            slot_rx_time = 0;
        }

        if(slot_rx_time == 1 && tx1_status == false)
        {
            UART_PutStr(USART3, "TX1: ");
            UART_PutStr(USART3, uart1_rx);
            tx1_status = true;
            memset(uart1_rx, 0 , sizeof(uart1_rx));
        }
//
//
        if(slot_rx_time == 3 &&  tx2_status == false)
        {
          //  printf("ctrl,%x,TX1,open,\n", STM32_UUID[0]&0xFFFFFF);
            UART_PutStr(USART3, "TX2: ");
            tx2_status = true;
        }

        if(slot_rx_time == 5 &&  ping_status == false)
        {
            ping_status = true;
            UART_PutStr(USART3, "PING: ");
            UART_PutStr(USART3, uart1_rx);
            memset(uart1_rx, 0 , sizeof(uart1_rx));

        }
    }
}
