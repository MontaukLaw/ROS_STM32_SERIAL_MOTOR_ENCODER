#include "sys.h"
#include "usart.h"	  
#include "lcd.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif


//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
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
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
volatile u8 ifTopicDefine =0;


void uart_init(u32 bound){
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
    //USART1_RX	  GPIOA.10��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
     //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

}


volatile u8 usartStatus=0;
volatile u8 frameBuffer[50];
volatile u8 bufferIndex = 0;
//u8 frameType = 0;
extern u8 ledDisp[];

void printCharArray(const char * content, u16 length){
    u16 i;
    for(i=0;i<length;i++){
        USART_SendData(USART1, content[i]);//�򴮿�1��������
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
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
                //���������İ�
                if( msgLength == 0){
                    if(frameBuffer[2] == 0x0b){
                        ifStartToPublish = 0;
                    }else{
                        ifTopicDefine = 1; 
                    }
                    
                }else{
                    if(frameBuffer[2] == 0x64){
                        //temp = 0;
                        //�������ݴ�frameBuffer[8]��ʼ                        
                        getDesirePWM(8);

                        setWalkingmotorSpeed( mbedODOMMotorControlMsg ); 
                        
                    }
                    //����������ݰ�����
                    //���tocpi_id��0A, ��ʾ��ʱ��֡/����˵ͬ��֡, ֱ���ӵ�
                    //�����0x64, ����λ���Ĺ�����publisher����Ϣ,��Ҫ�������
                    temp = 0;
                }

                usartStatus = READY;
                flushDisp();
                break;
            
            
            }                  
        
    }
}


#endif	

