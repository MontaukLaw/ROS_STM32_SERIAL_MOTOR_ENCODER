#ifndef __TOOLS_H
#define __TOOLS_H

#include "sys.h"
#include <math.h>


extern char uartTxBuffer[];

#define ASCII_BASE   	48



u16 lengthOfInt(long long intNumber);

u8 publishTopic(long long leftEncoderCounter, long long rightEncoderCounter, long long timer, u16 topicID);


#endif


