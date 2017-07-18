#include "tools.h"

u16 lengthOfInt(long long intNumber){
	
    u16 length;
    if(intNumber <= 9 && intNumber > 0){
        length=1;
    }else{
        length=0;
    }
    
    while(intNumber>=1){
        intNumber=intNumber/10;
        ++length;
   }
   return length;
    
}


u16 filEncoderCounter(u8 offSet, long long encoderCounter){
    u16 tempLength,i;   
    long long tempLong;
    long long tempCounter;
    tempCounter = encoderCounter;
    tempLength = lengthOfInt(encoderCounter);

   
    for(i = 0; i < tempLength; i++){
        tempLong = pow(10, tempLength - 1 - i);
        uartTxBuffer[offSet + i] = ASCII_BASE + (u8)(tempCounter / tempLong);
        tempCounter = tempCounter - tempLong * (u8)(tempCounter / tempLong);
    }
    
    return tempLength;
    
    //offSet = offSet + tempLength;
    
}


u8 publishTopic(long long leftEncoderCounter, long long rightEncoderCounter, long long timer, u16 topicID){
    u16 msgLength;
    u16 msgLengthChk;
    u8 frameChk;
    u16 msgSum;
	u32 publisherLength;
    u16 tempLength=0;
	u8 offSet = 0;
    long long tempLong;
    long long tempCounter;
    u16 i;
	//msg + 2 "," + msgLenth 4 bytes
    msgLength= 4 + 2 + lengthOfInt(leftEncoderCounter) + lengthOfInt(rightEncoderCounter) + lengthOfInt(timer);

    uartTxBuffer[offSet]=0xff;
	offSet++;
    uartTxBuffer[offSet]=0xfe;
    offSet++;
    uartTxBuffer[offSet] = (u8)(msgLength);
    uartTxBuffer[offSet+1] = (u8)(msgLength>>8);

    offSet = offSet + 2;	
    
	msgLengthChk = 255 - ( (( msgLength&255 ) + ( msgLength >>8 )) % 256 );

	uartTxBuffer[offSet] = msgLengthChk;
	offSet++;
	
	uartTxBuffer[offSet] = (u8)(topicID);
	uartTxBuffer[offSet+1] = (u8)(topicID>>8);
    
	offSet = offSet + 2;
	
	publisherLength = msgLength - 4;	
	
	uartTxBuffer[offSet] =  (u8)(publisherLength);
	uartTxBuffer[offSet+1] =  (u8)(publisherLength>>8);
	uartTxBuffer[offSet+2] =  (u8)(publisherLength>>16);
	uartTxBuffer[offSet+3] =  (u8)(publisherLength>>24);
 	offSet = offSet + 4;
	
    tempLength = filEncoderCounter(offSet,leftEncoderCounter);    
    offSet = offSet + tempLength;
    uartTxBuffer[offSet] =  0x2c;
    offSet++;
    
    tempLength = filEncoderCounter(offSet,rightEncoderCounter);    
    offSet = offSet + tempLength;    
    uartTxBuffer[offSet] =  0x2c;
    offSet++;

    tempLength = filEncoderCounter(offSet,timer);    
    offSet = offSet + tempLength;    

    //msg_checksum = 255 - ( ((topic&255) + (topic>>8) + sum([ord(x) for x in msg]))%256 )
    for(i=5; i<offSet ;i++){
        msgSum = msgSum + uartTxBuffer[i];
    }
    
    frameChk = 255 - msgSum % 256;
    uartTxBuffer[offSet] = frameChk;
    offSet++;
	return offSet;
}
