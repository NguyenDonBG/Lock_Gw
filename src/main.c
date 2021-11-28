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

extern char Rx_bufArr[20];
extern char uart1_rx[20];

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

    if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0 && status_button == true)
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
void Task_Pair_Connect(void)
{
    Button_Detect_Event(GPIOA, BUTTON_PAIR, &status_pair);
    while(status_pair == true)
    {
         LED_OFF();
         printf("pa,%x,1,1,\r\n",STM32_UUID[0]&0xFFFF);
         status_pair = false;
    }
    LED_ON();
}

int main(void)
{
    Gpio_Init();

    UART1_Init_A9A10(1200);
    UART3_Config(115200);
    LED_ON();
    GPIO_SetBits(GPIOA, RF_SET_PIN);
    GPIO_ResetBits(GPIOA, RF_CS_PIN);
    char type[5];
    char ID_get_way[7];
    char ID_node[7];
    char mess[5];
    char ID_GW[5];
    sprintf(ID_GW, "%x", STM32_UUID[0]&0xFFFF);
    while(1)
    {
        memset(type, 0, sizeof(type));
        memset(ID_get_way, 0, sizeof(ID_get_way));
        memset(ID_node, 0, sizeof(ID_node));
        memset(mess, 0, sizeof(mess));
        memset(uart1_rx, 0 , sizeof(uart1_rx));
        Task_Pair_Connect();
        if(strlen(Rx_bufArr) > 0)
        {
            while(strchr(Rx_bufArr, '\n') == NULL);
            Process_Message(Rx_bufArr, type, ID_get_way, ID_node, mess);
            if(strstr(type, "ct") != NULL && strchr(ID_node, '1') != NULL && strchr(mess, '1') != NULL)
            {
                printf("ct,%x,1,1,\n", STM32_UUID[0]&0xFFFF);
            }
            memset(Rx_bufArr, 0, sizeof(Rx_bufArr));
        }

        if(strlen(uart1_rx) != 0)
        {
           // UART_PutStr(USART3, uart1_rx);
            while(strstr(uart1_rx, "\n") == NULL);
            Process_Message(Rx_bufArr, type, ID_get_way, ID_node, mess);
            if(strstr(ID_GW, ID_get_way) != NULL)
            {
                UART_PutStr(USART3, uart1_rx);
                memset(uart1_rx, 0 , strlen(uart1_rx));
            }
            memset(Rx_bufArr, 0, strlen(Rx_bufArr));
            memset(ID_get_way, 0, strlen(ID_get_way));

        }
    }
}
