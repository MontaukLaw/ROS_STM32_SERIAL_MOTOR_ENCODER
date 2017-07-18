#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"
#include "tools.h"
#include "usart.h"

void TIM3_Int_Init(u16 arr,u16 psc);
extern const char TIME_TOPIC[];
extern char uartTxBuffer[];
extern u8 ifStartToPublish;

typedef void (*tim_oc_init_t)(TIM_TypeDef*, TIM_OCInitTypeDef*);
typedef void (*tim_oc_preloadcfg)(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
typedef void (*tim_set_compare)(TIM_TypeDef* TIMx, uint16_t Compare3);

typedef struct tim_func {
    tim_oc_init_t     oc_init;          /**< Timer oc init function. */
    tim_oc_preloadcfg oc_preloadcfg;    /**< Timer oc preload configure function. */
    tim_set_compare   set_compare;      /**< Timer set compare status function. */
} tim_func_t;
 
extern const tim_func_t g_tim_func[];   /**< Global timer function array. */

#define tim_oc_init(tim, ch, oc)                      \
    if (ch > 0 && ch <= CONFIG_TIME_CHANNEL_NUM) {    \
        g_tim_func[ch-1].oc_init(tim, oc);            \
    }

#define tim_oc_preloadcfg(tim, ch, preload)           \
    if (ch > 0 && ch <= CONFIG_TIME_CHANNEL_NUM) {    \
        g_tim_func[ch-1].oc_preloadcfg(tim, preload); \
    }

#define tim_set_compare(tim, ch, duty)                \
    if (ch > 0 && ch <= CONFIG_TIME_CHANNEL_NUM) {    \
        g_tim_func[ch-1].set_compare(tim, duty);      \
    }

   
    
#endif

