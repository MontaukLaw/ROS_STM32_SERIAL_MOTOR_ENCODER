#include "sys.h"
#include "usart.h"	  
#include "lcd.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
volatile u8 ifTopicDefine =0;


void uart_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
    //USART1_RX	  GPIOA.10初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
     //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART1, &USART_InitStructure); //初始化串口1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART1, ENABLE);                    //使能串口1 

}


volatile u8 usartStatus=0;
volatile u8 frameBuffer[50];
volatile u8 bufferIndex = 0;
//u8 frameType = 0;
extern u8 ledDisp[];

void printCharArray(const char * content, u16 length){
    u16 i;
    for(i=0;i<length;i++){
        USART_SendData(USART1, content[i]);//向串口1发送数据
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
}

void flushDisp(void){
    //ledDisp[2] = usartStatus + 48;
}

volatile u16 desireLeftPWM = 0;
volatile u16 desireRightPWM = 0;
u8 desireLeftDirection = 0 ;
u8 desireRightDirection = 0 ;

extern struct MotorControlMsg mbedODOMMotorControlMsg; 
 
void getDesirePWM(u8 offset){
    volatile u8 byteCounter = 0;
    mbedODOMMotorControlMsg.leftDirection = frameBuffer[offset] - 0x30;
    offset = offset + 2;
    mbedODOMMotorControlMsg.rightDirection = frameBuffer[offset] - 0x30;
    offset = offset + 2;
    mbedODOMMotorControlMsg.pLeft = 0;
    mbedODOMMotorControlMsg.pRight = 0 ;
    
    while(frameBuffer[offset] != 0x2C){
        mbedODOMMotorControlMsg.pLeft = mbedODOMMotorControlMsg.pLeft * 10 + (frameBuffer[offset] - 0x30);
        offset ++;
    }
    offset ++;
    
    while(offset < frameBuffer[0] + 4){
        mbedODOMMotorControlMsg.pRight = mbedODOMMotorControlMsg.pRight * 10 + (frameBuffer[offset] - 0x30);
        offset ++;
    }


}

volatile u8 msgLength=0;
volatile u8 tempC=0; 

   
void USART1_IRQHandler(void){
    volatile u8 Res;
    static u8 temp=0;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        Res = (u8)USART_ReceiveData(USART1);
        
        switch( usartStatus ){          
            case READY:
                if( Res == 0xff ){
                    usartStatus = FRAME_START;
                    flushDisp(); 
                    ledDisp[5]= 48 + tempC;
                    tempC++;                    
                }                
                break;
                            
            case FRAME_START:
                if( Res == 0xfe ){
                    usartStatus = VERSION_CORRECT;
                    bufferIndex = 0;
                    flushDisp();
                }
                break;
 
            case VERSION_CORRECT:
                frameBuffer[bufferIndex] = Res;
                bufferIndex++;
                if( bufferIndex == 2){
                    //bufferIndex=0;
                    msgLength=frameBuffer[0];
                    if( msgLength == 0){
                        msgLength = 0;
                    }
                    usartStatus = FRAME_LENGTH_CHK;
                    flushDisp();
                } 
                break;          
                            
            case FRAME_LENGTH_CHK:               
                usartStatus = FRAME_TOPIC_ID;
                flushDisp();
                break;
                          
            case FRAME_TOPIC_ID:
                frameBuffer[bufferIndex] = Res;
                bufferIndex++;
                if( bufferIndex == 4){
                    flushDisp();
                    if(msgLength > 0){
                        usartStatus = FRAME_MESSAGE;
                        temp = msgLength + 4;
                    }else{
                        usartStatus = FRAME_CHECK_SUM;;
                    }
                }
                break;
                               
            case FRAME_MESSAGE:   
            
                if( bufferIndex < temp ){                    
                    frameBuffer[bufferIndex] = Res;    
                    bufferIndex++; 
                    if(bufferIndex == temp){
                        usartStatus = FRAME_CHECK_SUM;                        
                        flushDisp();
                    }                    
                }          
                break;   

            case FRAME_CHECK_SUM:
                //如果是命令的包
                if( msgLength == 0){
                    if(frameBuffer[2] == 0x0b){
                        ifStartToPublish = 0;
                    }else{
                        ifTopicDefine = 1; 
                    }
                    
                }else{
                    if(frameBuffer[2] == 0x64){
                        //temp = 0;
                        //正经内容从frameBuffer[8]开始                        
                        getDesirePWM(8);

                        setWalkingmotorSpeed( mbedODOMMotorControlMsg ); 
                        
                    }
                    //这里进行数据包分析
                    //如果tocpi_id是0A, 表示是时间帧/或者说同步帧, 直接扔掉
                    //如果是0x64, 是上位机的过来的publisher的消息,需要拆包分析
                    temp = 0;
                }

                usartStatus = READY;
                flushDisp();
                break;
            
            
            }                  
        
    }
}


#endif	

