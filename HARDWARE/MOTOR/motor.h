#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"
#include "common.h"

#define CONFIG_MOTOR_ENCODER_NUM    2


//×óÂÖ
#define MOTO_LF_EN_PORT             GPIOE       /** PE13 ×óÂÖÇ°½ø */
#define MOTO_LF_EN_PIN              GPIO_Pin_13

#define MOTO_LF_EN_ID              1           /**< PE13 is TIM1_CH3 */
#define MOTO_LF_EN_CHN             3
//#define MOTO_LF_EN                 (GET_TIM(MOTO_LF_EN_ID))

#define MOTO_LB_EN_PORT             GPIOE       /** PE14 is ×óÂÖºóÍË. */
#define MOTO_LB_EN_PIN              GPIO_Pin_14

#define MOTO_LB_EN_ID              1           /**< PE13 is TIM1_CH4 */
#define MOTO_LB_EN_CHN             4
//#define MOTO_LB_EN                 (GET_TIM(MOTO_LB_EN_ID))

#define MOTO_L_PWM_PORT            GPIOB       /** PB6 is ×óÂÖ pwm. */
#define MOTO_L_PWM_PIN             GPIO_Pin_6

#define MOTO_L_PWM_ID              4           /**< PB6 is TIM4_CH1, */
#define MOTO_L_PWM_CHN             1
//#define MOTO_L_PWM                 (GET_TIM(MOTO_L_PWM_ID))

//ÓÒÂÖ
#define MOTO_RB_EN_PORT            GPIOB       /** PB7 is ÓÒÂÖÇ°½ø. */
#define MOTO_RB_EN_PIN             GPIO_Pin_7

#define MOTO_RB_EN_ID              4
#define MOTO_RB_EN_CHN             2           /**< PB7 is TIM4_CH2 */
//#define MOTO_RB_EN                 (GET_TIM(MOTO_RB_EN_ID)) 
 
#define MOTO_RF_EN_PORT            GPIOB       /** PB9 ÊÇÓÒÂÖºóÍË. */
#define MOTO_RF_EN_PIN             GPIO_Pin_9

#define MOTO_RF_EN_ID              4
#define MOTO_RF_EN_CHN             4           /**< PB9 is TIM4_CH4 */
//#define MOTO_RF_EN                 (GET_TIM(MOTO_RF_EN_ID))

#define MOTO_R_PWM_PORT            GPIOB       /** PB8 ÊÇÓÒÂÖ pwm. */
#define MOTO_R_PWM_PIN             GPIO_Pin_8

#define MOTO_R_PWM_ID              4
#define MOTO_R_PWM_CHN             3           /**< PB8 is TIM4_CH3 */
//#define MOTO_R_PWM                 (GET_TIM(MOTO_R_PWM_ID))

//±àÂëÆ÷
#define ENCODER_L1_PORT             GPIOD
#define ENCODER_L1_PIN              GPIO_Pin_6
#define ENCODER_L1_EXTI             6

#define ENCODER_L2_PORT             GPIOD
#define ENCODER_L2_PIN              GPIO_Pin_4
#define ENCODER_L2_EXTI             4

#define ENCODER_R1_PORT             GPIOD
#define ENCODER_R1_PIN              GPIO_Pin_2
#define ENCODER_R1_EXTI             2

#define ENCODER_R2_PORT             GPIOD
#define ENCODER_R2_PIN              GPIO_Pin_3
#define ENCODER_R2_EXTI             3

#define ODOMETER_EST_PULSE_PER_METER  138867UL

enum motorCtrlState_t {
    MOTOR_CTRL_STATE_RELEASE = 0,
    MOTOR_CTRL_STATE_FORWARD = 1,
    MOTOR_CTRL_STATE_BACKWARD = 2,
    MOTOR_CTRL_STATE_BRAKE = 3,
};

typedef enum speed_monitor {
    MOTOR_SPEED_MONI_NONE,  /**< No motor speed stall monitor. */
    MOTOR_SPEED_MONI_BUMP,  /**< Motor speed stall mapping as bump action. */
    MOTOR_SPEED_MONI_STALL, /**< Motor speed stall mapping as stall action. */
} speed_monitor_t;

#define CONFIG_MOTOR_PWM_PERIOD (10000)

#define WALKINGMOTOR_CNT        2

#define WALKINGMOTOR_L_ID       0
#define WALKINGMOTOR_R_ID       1

struct MotorControlMsg {    
    _u16 pLeft;
    _u16 pRight;
    _u16 leftDirection;
    _u16 rightDirection;
};

#define FORWARD 1
#define BACKWARD 2
#define BREAK 3
#define REALEASE 4

#define LEFT_MOTOR 1
#define RIGHT_MOTOR 0

void initMotor(void);

void setWalkingmotorSpeed(struct MotorControlMsg motorControlMsg);


#endif
