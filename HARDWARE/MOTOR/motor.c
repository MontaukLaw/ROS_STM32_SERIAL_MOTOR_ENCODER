#include "motor.h"

//PE13 ����ǰ��
//PE14 ��·����
//PB6  ����PWM
//PB7  ����ǰ��
//PB9  ���ֺ���
//PB8  ����PWM

#if 0
static const motor_cfg_ydy_t _motor_cfg_ydy[]={
    {   /**< Left walking motor configure. */
        {MOTO_LF_EN_PORT, MOTO_LF_EN_PIN, MOTO_LF_EN, MOTO_LF_EN_CHN},
        {MOTO_LB_EN_PORT, MOTO_LB_EN_PIN, MOTO_LB_EN, MOTO_LB_EN_CHN},
        {MOTO_L_PWM_PORT, MOTO_L_PWM_PIN, MOTO_L_PWM, MOTO_L_PWM_CHN},
        {NULL, 0, LOW},
        {ENCODER_L1_PORT, ENCODER_L1_PIN, ENCODER_L1_EXTI},
        {ENCODER_L2_PORT, ENCODER_L2_PIN, ENCODER_L2_EXTI},
        ODOMETER_EST_PULSE_PER_METER,
    },

};

static _u32 _encoder1TicksDelta[WALKINGMOTOR_CNT];   
static _u32 _encoder2TicksDelta[WALKINGMOTOR_CNT];      

#endif

uint8_t leftMotoDirection;
uint8_t rightMotoDirection;

void leftForward(void){
    GPIO_SetBits(GPIOE,GPIO_Pin_13); 
    GPIO_ResetBits(GPIOE,GPIO_Pin_14); 
}

void rightForward(void){
    GPIO_SetBits(GPIOB,GPIO_Pin_7); 
    GPIO_ResetBits(GPIOB,GPIO_Pin_9); 
}

void setLeftPWM(u16 pwm){
	TIM_SetCompare1(TIM4,pwm);
}

void setRightPWM(u16 pwm){
	TIM_SetCompare3(TIM4,pwm);
}

void setWalkingmotorSpeed(struct MotorControlMsg motorControlMsg)
{
    if(motorControlMsg.pLeft > CONFIG_MOTOR_PWM_PERIOD||motorControlMsg.pRight > CONFIG_MOTOR_PWM_PERIOD) {
        return;
    }

    switch(motorControlMsg.leftDirection) {
        case FORWARD:
            GPIO_SetBits(GPIOE,GPIO_Pin_13); 
            GPIO_ResetBits(GPIOE,GPIO_Pin_14);
            setLeftPWM(motorControlMsg.pLeft);
            //left_pwm.write(motorControlMsg.pLeft);
            break;
        case BACKWARD:
            GPIO_ResetBits(GPIOE,GPIO_Pin_13); 
            GPIO_SetBits(GPIOE,GPIO_Pin_14);
            setLeftPWM(motorControlMsg.pLeft);
            break;
        case BREAK:
            GPIO_ResetBits(GPIOE,GPIO_Pin_13); 
            GPIO_ResetBits(GPIOE,GPIO_Pin_14);
            setLeftPWM(10000);
            break;
        case REALEASE:
            GPIO_SetBits(GPIOE,GPIO_Pin_13); 
            GPIO_SetBits(GPIOE,GPIO_Pin_14);
            setLeftPWM(10000);
            break;
    }

    switch(motorControlMsg.rightDirection) {
        case FORWARD:
            GPIO_SetBits(GPIOB,GPIO_Pin_7); 
            GPIO_ResetBits(GPIOB,GPIO_Pin_9); 
            setRightPWM(motorControlMsg.pRight);
            break;
        case BACKWARD:
            GPIO_ResetBits(GPIOB,GPIO_Pin_7); 
            GPIO_SetBits(GPIOB,GPIO_Pin_9); 
            setRightPWM(motorControlMsg.pRight);
            break;
        case BREAK:
            GPIO_ResetBits(GPIOB,GPIO_Pin_7); 
            GPIO_ResetBits(GPIOB,GPIO_Pin_9);
            setRightPWM(10000);
            break;
        case REALEASE:
            GPIO_SetBits(GPIOB,GPIO_Pin_7); 
            GPIO_SetBits(GPIOB,GPIO_Pin_9);
            setRightPWM(10000);
            break;
    }
}


void TIM4_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ�ܶ�ʱ��4ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
   //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_8; //TIM4_CH1/TIM4_CH3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
 
   //��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM3 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	//TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
   
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
 	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
 
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM3
    
	//TIM_SetCompare1(TIM4,300);
    //TIM_SetCompare2(TIM4,120);
    //TIM_SetCompare3(TIM4,4000);
    //TIM_SetCompare4(TIM4,400);
    //TIM_SetCompare1(TIM4,500);
}

void motorTest(void){
    struct MotorControlMsg motorControlMsg={1000,1000,FORWARD,FORWARD};
    
    setWalkingmotorSpeed(motorControlMsg);
}

void initMotor(void){    
    
    //ǰ�����˿���, ��4����, PE13/PE14/PB7/PB9
    GPIO_InitTypeDef  GPIO_InitStructure; 	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_9;	 //LED0-->PB.5 �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
	    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;	  //LED1-->PE.5 �˿�����, �������
    GPIO_Init(GPIOE, &GPIO_InitStructure);	  				  //������� ��IO���ٶ�Ϊ50MHz
   
    //���ܲ���:      
    //leftForward();
    //rightForward();
    
    //�����ǳ�ʼ��pwm����
    TIM4_PWM_Init(4999,0);
        
    //����
    //motorTest();

}



#if 0

static void _init_motor_pwm(const pwm_port_t *pwm)
{
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_OCInitTypeDef       tim_oc;

    if (pwm == NULL || pwm->port == NULL) {
        return ;
    }

    pinMode(pwm->port, pwm->pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(pwm->port,  pwm->pin, Bit_SET);

    TIM_CtrlPWMOutputs(pwm->tim, DISABLE);
    TIM_ARRPreloadConfig(pwm->tim, DISABLE);
    TIM_Cmd(pwm->tim, DISABLE);

    /* Initialize motor control pwm. */
    tim_base.TIM_Period = (CONFIG_MOTOR_PWM_PERIOD - 1);
    tim_base.TIM_Prescaler = 0;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(pwm->tim, &tim_base);

    /* Motor PWM ouput channel configure. */
    tim_oc.TIM_OCMode      = TIM_OCMode_PWM2;
    tim_oc.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc.TIM_Pulse       = CONFIG_MOTOR_PWM_PERIOD;
    //tim_oc.TIM_OCPolarity  = TIM_OCPolarity_Low;
    tim_oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    
    tim_oc_init(pwm->tim, pwm->tim_ch, &tim_oc);
    tim_oc_preloadcfg(pwm->tim, pwm->tim_ch, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(pwm->tim, ENABLE);
    TIM_ARRPreloadConfig(pwm->tim, ENABLE);
    TIM_Cmd(pwm->tim, ENABLE);
    
    //TIM_SetCompare3(TIM4,200);
}

static void motor_set_duty(const pwm_port_t *pwm, int duty)
{
    if (pwm == NULL) {
        return ;
    }

    duty = abs(duty);
    if (duty > CONFIG_MOTOR_PWM_PERIOD) {
        duty = CONFIG_MOTOR_PWM_PERIOD;
    }
    tim_set_compare(pwm->tim, pwm->tim_ch, CONFIG_MOTOR_PWM_PERIOD - duty);

    if (duty > 0) {
        pinMode(pwm->port, pwm->pin, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    }
}

static void _set_walkingmotor_forward(_u8 id, int duty)
{
    const pwm_port_t *pwm, *forwardEN, *backwardEN;
    pwm = &_motor_cfg_ydy[id].pwm;
    forwardEN = &_motor_cfg_ydy[id].fw_en;
    backwardEN = &_motor_cfg_ydy[id].bw_en;

    
    if (id >= WALKINGMOTOR_CNT) {
        return ;
    }
      
    //forward enable:
    pinMode(forwardEN->port, forwardEN->pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(forwardEN->port, forwardEN->pin, Bit_SET);
    
    //backward disable:
    pinMode(backwardEN->port, backwardEN->pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(backwardEN->port, backwardEN->pin, Bit_RESET);

    motor_set_duty(pwm, duty);
        
    return ;
}

void init_walkingmotor(void)
{
    _u8 i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);

    for (i = 0; i < _countof(_motor_cfg_ydy); i++) {

        _init_motor_pwm(&_motor_cfg_ydy[i].pwm);
        _init_motor_pwm(&_motor_cfg_ydy[i].fw_en);
        _init_motor_pwm(&_motor_cfg_ydy[i].bw_en);

        if (_motor_cfg_ydy[i].oc_mon.port != NULL) {
            pinMode(_motor_cfg_ydy[i].oc_mon.port, _motor_cfg_ydy[i].oc_mon.pin, GPIO_Mode_IN_FLOATING, GPIO_Speed_10MHz);
        }
    }

    _set_walkingmotor(WALKINGMOTOR_L_ID, 0, MOTOR_CTRL_STATE_RELEASE);
    _set_walkingmotor(WALKINGMOTOR_R_ID, 0, MOTOR_CTRL_STATE_RELEASE);
    //memset(_motorSpeedMm, 0, sizeof(_motorSpeedMm));
    //memset(speedLastErr, 0, sizeof(speedLastErr));
    //memset(speedErri, 0, sizeof(speedErri));

}

//�������ⲿ�ж�����
void encoder_l1_exti_cb(void)
{
    ++_encoder1TicksDelta[WALKINGMOTOR_L_ID];
}

void encoder_l2_exti_cb(void)
{
    ++_encoder2TicksDelta[WALKINGMOTOR_L_ID];
}


void encoder_r1_exti_cb(void)
{
    ++_encoder1TicksDelta[WALKINGMOTOR_R_ID];
}

void encoder_r2_exti_cb(void)
{
    ++_encoder2TicksDelta[WALKINGMOTOR_R_ID];
}

static void init_extix(void)
{
    _u8 i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    for (i = 0; i < _countof(_motor_cfg_ydy); i++) {
        pinMode(_motor_cfg_ydy[i].encoder1.port, _motor_cfg_ydy[i].encoder1.pin, GPIO_Mode_IPU, GPIO_Speed_50MHz);
        pinMode(_motor_cfg_ydy[i].encoder2.port, _motor_cfg_ydy[i].encoder2.pin, GPIO_Mode_IPU, GPIO_Speed_50MHz);
    }

    GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg_ydy[WALKINGMOTOR_L_ID].encoder1.port), _motor_cfg_ydy[WALKINGMOTOR_L_ID].encoder1.exti);
    GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg_ydy[WALKINGMOTOR_R_ID].encoder1.port), _motor_cfg_ydy[WALKINGMOTOR_R_ID].encoder1.exti);
    exti_reg_callback(_motor_cfg_ydy[WALKINGMOTOR_L_ID].encoder1.exti, EXTI_Trigger_Rising_Falling, encoder_l1_exti_cb);
    exti_reg_callback(_motor_cfg_ydy[WALKINGMOTOR_R_ID].encoder1.exti, EXTI_Trigger_Rising_Falling, encoder_r1_exti_cb);

    GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg_ydy[WALKINGMOTOR_L_ID].encoder2.port), _motor_cfg_ydy[WALKINGMOTOR_L_ID].encoder2.exti);
    GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg_ydy[WALKINGMOTOR_R_ID].encoder2.port), _motor_cfg_ydy[WALKINGMOTOR_R_ID].encoder2.exti);
    exti_reg_callback(_motor_cfg_ydy[WALKINGMOTOR_L_ID].encoder2.exti, EXTI_Trigger_Rising_Falling, encoder_l2_exti_cb);
    exti_reg_callback(_motor_cfg_ydy[WALKINGMOTOR_R_ID].encoder2.exti, EXTI_Trigger_Rising_Falling, encoder_r2_exti_cb);

}

void set_walkingmotor_speed(_s32 lSpeed, _s32 rSpeed)
{
    if (abs(lSpeed) - abs(_motorSpeedMm[WALKINGMOTOR_L_ID]) < 0) {
        _stallDetectorFilter[WALKINGMOTOR_L_ID] = 0;
    }
    if (abs(rSpeed) - abs(_motorSpeedMm[WALKINGMOTOR_R_ID]) < 0) {
        _stallDetectorFilter[WALKINGMOTOR_R_ID] = 0;
    }
    _motorSpeedMm[WALKINGMOTOR_L_ID] = lSpeed;
    _motorSpeedMm[WALKINGMOTOR_R_ID] = rSpeed;
}

void motoInit(void){
    _u8 i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);

    pinMode(MOTO_EN_PORT, MOTO_EN_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(MOTO_EN_PORT,  MOTO_EN_PIN, Bit_SET);

    for (i = 0; i < 2; i++) {

        _init_motor_pwm(&_motor_cfg_ydy[i].pwm);
        _init_motor_pwm(&_motor_cfg_ydy[i].fw_en);
        _init_motor_pwm(&_motor_cfg_ydy[i].bw_en);

        if (_motor_cfg_ydy[i].oc_mon.port != NULL) {
            pinMode(_motor_cfg_ydy[i].oc_mon.port, _motor_cfg_ydy[i].oc_mon.pin, GPIO_Mode_IN_FLOATING, GPIO_Speed_10MHz);
        }
    }

    _set_walkingmotor(WALKINGMOTOR_L_ID, 0, MOTOR_CTRL_STATE_RELEASE);
    _set_walkingmotor(WALKINGMOTOR_R_ID, 0, MOTOR_CTRL_STATE_RELEASE);

}

#endif
