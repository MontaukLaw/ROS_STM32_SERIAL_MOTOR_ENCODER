#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#define READY 0
#define FRAME_START  1
#define VERSION_CORRECT  2
#define FRAME_LENGTH_CHK 3
#define FRAME_TOPIC_ID 4
#define FRAME_MESSAGE 5
#define FRAME_CHECK_SUM 6

#define ID_PUBLISHER                0
#define ID_SUBSCRIBER               1
#define ID_SERVICE_SERVER           2
#define ID_SERVICE_CLIENT           4 
#define ID_PARAMETER_REQUEST        6 
#define ID_LOG                      7 
#define ID_TIME                     10
#define ID_TX_STOP                  11

#define FRAME_TYPE_REQUEST_TOPIC    1
#define FRAME_TYPE_SYNC             2

void printCharArray(const char * content, u16 length);
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
extern u8 ifStartToPublish;

//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);

#endif


