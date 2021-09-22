#ifndef __lowpowerMCU_H
#define __lowpowerMCU_H
#ifdef __cplusplus
    extern "C"{
#endif
#include "stm32f10x.h"
void Sleep_MCU_Config(void);
void Sleep_MCU_Run(void);
#ifdef __cplusplus
}
#endif

#endif
