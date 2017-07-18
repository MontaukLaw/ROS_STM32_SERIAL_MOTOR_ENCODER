#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

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
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
extern u8 ifStartToPublish;

//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);

#endif


