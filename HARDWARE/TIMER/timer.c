#include "timer.h"
#include "lcd.h"
#include "motor.h"

u8 ledDisp[]={'S',':','0','T',':','b','c','d'};
const char TIME_TOPIC[] = {0xFF,0xFE,0x08,0x00,0xF7,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF5}; 
volatile u8 tim3HeartBeatCounter;

extern long long leftWheelCounter;
extern long long rightWheelCounter;

const tim_func_t g_tim_func[] = {
    {TIM_OC1Init, TIM_OC1PreloadConfig, TIM_SetCompare1},
    {TIM_OC2Init, TIM_OC2PreloadConfig, TIM_SetCompare2},
    {TIM_OC3Init, TIM_OC3PreloadConfig, TIM_SetCompare3},
    {TIM_OC4Init, TIM_OC4PreloadConfig, TIM_SetCompare4},
};

void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx					 
}

long long timeNow = 0;
//��ʱ��3�жϷ������
volatile u16 pwmChangeCounter = 0;

struct MotorControlMsg mbedODOMMotorControlMsg = {1000,1000,FORWARD,FORWARD};

void testMotor(void){
    pwmChangeCounter = pwmChangeCounter + 1;
        
    if(pwmChangeCounter == 5000){
        pwmChangeCounter = 0;
    }
        
    mbedODOMMotorControlMsg.pLeft = pwmChangeCounter;
    mbedODOMMotorControlMsg.pRight = pwmChangeCounter;
        
    //motorControlMsg.pRight= 10000-pwmChangeCounter;
    setWalkingmotorSpeed( mbedODOMMotorControlMsg ); 

}

void TIM3_IRQHandler(void)   //TIM3�ж�
{
    u8 txBufferSize;

	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
	{
        
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־ 
		    timeNow++;
            if(ifStartToPublish){
                
                txBufferSize = publishTopic(leftWheelCounter,rightWheelCounter,timeNow,125);
                printCharArray(uartTxBuffer,txBufferSize);
            
                tim3HeartBeatCounter++;
                if( tim3HeartBeatCounter > 200){
                    tim3HeartBeatCounter = 0;
                    printCharArray(TIME_TOPIC,sizeof(TIME_TOPIC));
                    //LCD_ShowString(60,50,200,16,16,ledDisp);
                }       
            }
            
           
            
	}
}
