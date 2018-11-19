
#ifndef  __Timer_Eval_h__
#define  __Timer_Eval_h__

#include "main.h"

#define SYS_TICK_PER_SECOND     100000UL

void Timer4_Configuration(void);

uint32_t get_current_tick(void);

uint32_t get_synchro_time(void);

void set_synchro_time(uint32_t syn_time_value);

#endif

