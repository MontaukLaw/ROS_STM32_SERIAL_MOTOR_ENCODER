#ifndef __COMMON_H
#define __COMMON_H

#include "sys.h"


typedef struct _exti_cfg {
    GPIO_TypeDef *port;         /**< EXTI port. */
    _u16          pin;          /**< EXTI pin. */
    _u8           exti;         /**< external interrupt. */
} exti_port_t;


typedef struct _in_port {
    GPIO_TypeDef *port;         /**< input port. */
    _u16          pin;          /**< input pin. */
    _u8           level;        /**< input trigger level. */
} in_port_t;

/**
 @brief output definitions.
 */
typedef struct _out_port {
    GPIO_TypeDef *port;         /**< output port. */
    _u16          pin;          /**< output pin. */
    _u8           level;        /**< output default level. */
} out_port_t;

typedef struct _pwm_port {
    GPIO_TypeDef *port;         /**< pwm control port. */
    _u16          pin;          /**< pwm control pin. */
    TIM_TypeDef  *tim;          /**< pwm timer. */
    _u8           tim_ch;       /**< pwm channel. */
} pwm_port_t;

typedef struct _motor_cfg_ydy {
    pwm_port_t  fw_en;         /**< Forward control pwm port. */
    pwm_port_t  bw_en;         /**< Backward control pwm port. */
    pwm_port_t  pwm;         /**< Backward control pwm port. */
    in_port_t   oc_mon;         /**< Over-current monitoring port. */
    exti_port_t encoder1;       /**< Motor encoder odometer port. */
    exti_port_t encoder2;       /**< Motor encoder odometer port. */
    _u16        speed_factor;   /**< Odometer speed factor, in pulse per meter. */
} motor_cfg_ydy_t;

#define HIGH 1
#define LOW  0

#define pinSet   GPIO_WriteBit
#define pinRead  GPIO_ReadInputDataBit





#endif

