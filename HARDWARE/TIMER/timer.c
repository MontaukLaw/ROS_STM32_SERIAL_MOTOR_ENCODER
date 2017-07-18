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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

	TIM_Cmd(TIM3, ENABLE);  //使能TIMx					 
}

long long timeNow = 0;
//定时器3中断服务程序
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

void TIM3_IRQHandler(void)   //TIM3中断
{
    u8 txBufferSize;

	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{
        
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志 
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
