/*
 * uart.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Georgy Moshkin
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 */

#include "uart.h"

COM_TPort* myPortPtr;
COM_TPort myPort[COM_MAX_NUM];

void dummy485() {}

void COM_Select(uint8_t n)
{
	myPortPtr=&myPort[n];
}

void COM_Init(uint8_t n,
		UART_HandleTypeDef *huartPtr,
		void *rxBufPtr,
		uint16_t rxBufSize,
		uint16_t timeOut
		)
{

	COM_Select(n);
	myPortPtr->huartPtr = huartPtr;
	myPortPtr->ndtrPtr=NDTR_PTR(myPortPtr);
	myPortPtr->rxBufPtr= rxBufPtr;
	myPortPtr->rxBufSize = rxBufSize;
	myPortPtr->rxPos = 0;
	myPortPtr->timeOut = timeOut;
	myPortPtr->rxFail = false;
	myPortPtr->_rs485tx=dummy485;
	myPortPtr->_rs485rx=dummy485;

	memset((uint8_t *) rxBufPtr, (char) 0, rxBufSize);
	HAL_UART_Receive_DMA(huartPtr, (uint8_t *) rxBufPtr, rxBufSize);

	CLEAR_BIT(huartPtr->Instance->CR3, USART_CR3_EIE);
	CLEAR_BIT(huartPtr->Instance->CR1, USART_CR1_PEIE);
}

void COM_SetCallbacks485(void (*f485tx)(void), void (*f485rx)(void))
{
	myPortPtr->_rs485tx=f485tx;
	myPortPtr->_rs485rx=f485rx;
	myPortPtr->_rs485rx();
}


void COM_WaitTxDone(void)
{
	while (READ_BIT(MY_ISR(myPortPtr),MY_ISR_TC)==0){asm("nop");}
}


void COM_Write(void *pData, uint16_t Size)
{
	COM_WaitTxDone();
	myPortPtr->_rs485tx();
	HAL_UART_Transmit_DMA(myPortPtr->huartPtr, pData,Size);
	COM_WaitTxDone();
	myPortPtr->_rs485rx();
}


void COM_WriteFast(void *pData, uint16_t Size)
{
	COM_WaitTxDone();
	HAL_UART_Transmit_DMA(myPortPtr->huartPtr, pData,Size);
}

bool COM_ReadByteTimeout(uint8_t *b, uint16_t timeOut)
{
	uint16_t dmaPos=myPortPtr->rxBufSize-*myPortPtr->ndtrPtr;

	uint32_t tickStart=HAL_GetTick();
	while ( (dmaPos==myPortPtr->rxPos) && ((HAL_GetTick() - tickStart) < timeOut) )
	{
		dmaPos=myPortPtr->rxBufSize-*myPortPtr->ndtrPtr;
	}

	if (dmaPos!=myPortPtr->rxPos)
	{
		*b=(myPortPtr->rxBufPtr)[myPortPtr->rxPos];
		myPortPtr->rxPos++;
		if (myPortPtr->rxPos==myPortPtr->rxBufSize) {myPortPtr->rxPos=0;}
		return true;
	}
	myPortPtr->rxFail=true;
	return false;
}

bool COM_ReadByteFast(uint8_t *b)
{
	return COM_ReadByteTimeout(b,0);
}

bool COM_ReadByte(uint8_t *b)
{
	return COM_ReadByteTimeout(b,myPortPtr->timeOut);
}

bool COM_ReadTimeout(uint8_t *destBuffer, uint16_t needLen, uint16_t timeOut)
{
	uint16_t UART_BUFFER_SIZE=myPortPtr->rxBufSize;
	uint16_t rxPos=myPortPtr->rxPos;
	uint16_t dataLen=0;
	uint16_t dmaPos=UART_BUFFER_SIZE-*myPortPtr->ndtrPtr;
	uint16_t dmaPosOld=dmaPos;

	uint32_t tickStart=HAL_GetTick();
	do {
		dmaPosOld=dmaPos;
		dmaPos=UART_BUFFER_SIZE-*myPortPtr->ndtrPtr;

		if (dmaPos!=dmaPosOld) {tickStart=HAL_GetTick();} // new byte arrived

		if (dmaPos>rxPos) {	dataLen=dmaPos-rxPos; }

		if (dmaPos<rxPos)
		{
			dataLen=UART_BUFFER_SIZE-rxPos;
			dataLen+=dmaPos;
		}
	} while ( (dataLen<needLen) && ((HAL_GetTick() - tickStart) < timeOut) );

	if (dataLen>=needLen)
	{
		if(rxPos+needLen-1<UART_BUFFER_SIZE)
		{
			memmove((uint8_t *)&destBuffer[0],(uint8_t *)&(myPortPtr->rxBufPtr)[rxPos],needLen);
		}
		else
		{
			uint16_t tailLen=UART_BUFFER_SIZE-rxPos;
			uint16_t headLen=needLen-tailLen;
			memmove((uint8_t *)&destBuffer[0],(uint8_t *)&(myPortPtr->rxBufPtr)[rxPos],tailLen);
			memmove((uint8_t *)&destBuffer[tailLen],(uint8_t *)&(myPortPtr->rxBufPtr)[0],headLen);
		}

		rxPos+=needLen;
		if (rxPos>=UART_BUFFER_SIZE) { rxPos-=UART_BUFFER_SIZE;}

		myPortPtr->rxPos=rxPos;
		return true;
	}


	if (timeOut>0) myPortPtr->rxPos=dmaPos; // abandon unread bytes
	myPortPtr->rxFail=true;
	return false;
}

bool COM_ReadFast(uint8_t *destBuffer, uint16_t needLen)
{
	return COM_ReadTimeout(destBuffer, needLen, 0);
}

bool COM_Read(uint8_t *destBuffer, uint16_t needLen)
{
	return COM_ReadTimeout(destBuffer, needLen, myPortPtr->timeOut);
}

void COM_CleanRxFail(void)
{
	myPortPtr->rxFail=false;
}

bool COM_RxFail(void)
{
	return myPortPtr->rxFail;
}
