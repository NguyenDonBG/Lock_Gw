/*
**
**                           lowpowerMCU.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "lowpowerMCU.h"
ErrorStatus HSEStartUpStatus;

/**
  * @brief  Configures RTC clock source and prescaler.
  * @param  None
  * @retval None
  */
static void RTC_Configuration(void)
{
    /* RTC clock source configuration ------------------------------------------*/
    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset Backup Domain */
    BKP_DeInit();

    /* Enable the LSE OSC */
    RCC_LSEConfig(RCC_LSE_ON);
    /* Wait till LSE is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {
    }

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable the RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* RTC configuration -------------------------------------------------------*/
    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();

    /* Set the RTC time base to 1s */
    RTC_SetPrescaler(32767);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Enable the RTC Alarm interrupt */
    RTC_ITConfig(RTC_IT_ALR, ENABLE);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}

/**
  * @brief  Configures NVIC and Vector Table base location.
  * @param  None
  * @retval None
  */
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 2 bits for Preemption Priority and 2 bits for Sub Priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void SYSCLKConfig_STOP(void)
{
    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if(HSEStartUpStatus == SUCCESS)
    {
        /* Enable PLL */
        RCC_PLLCmd(ENABLE);

        /* Wait till PLL is ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while(RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
}
/**
  * @brief  Configures Standy mode and RTC
  * @param  None
  * @retval None
  */
void Sleep_MCU_Config(void)
{
     /* Enable PWR and BKP clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_BKP, ENABLE);
     /* Configure RTC clock source and prescaler */
    RTC_Configuration();
     /* NVIC configuration */
    NVIC_Configuration();
}

/**
  * @brief  Sleep mode low power
  * @param  None
  * @retval None
  */
void Sleep_MCU_Run(void)
{
    RTC_ClearFlag(RTC_FLAG_SEC);
    while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

    /* Alarm in 3 second */
    RTC_SetAlarm(RTC_GetCounter()+3);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    PWR_EnterSTANDBYMode();
    SYSCLKConfig_STOP();
}
