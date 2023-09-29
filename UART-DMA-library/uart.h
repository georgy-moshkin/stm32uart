/*
 * uart.h
 *
 *  Created on: Sep 12, 2023
 *      Author: Georgy Moshkin
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include "string.h"

#define STM32F1
//#define STM32F4
//#define STM32H7

#define COM_MAX_NUM 8

#ifdef STM32F1
#include "stm32f1xx_hal.h"
#define MY_ISR(p) p->huartPtr->Instance->SR
#define MY_ISR_TC USART_SR_TC
#define NDTR_PTR(p) &(p->huartPtr->hdmarx->Instance->CNDTR)
#endif

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#define MY_ISR(p) p->huartPtr->Instance->SR
#define MY_ISR_TC USART_SR_TC
#define NDTR_PTR(p) &(p->huartPtr->hdmarx->Instance->CNDTR)
#endif

#ifdef STM32H7
#include "stm32h7xx_hal.h"
#define MY_ISR(p) p->huartPtr->Instance->ISR
#define MY_ISR_TC USART_ISR_TC
#define NDTR_PTR(p) &(((DMA_Stream_TypeDef*)p->huartPtr->hdmarx->Instance)->NDTR)
#endif


typedef struct
{
	UART_HandleTypeDef* huartPtr;
	volatile uint32_t* ndtrPtr;
	uint8_t* rxBufPtr;
	uint16_t rxBufSize;
	uint16_t rxPos;
	uint16_t timeOut;
	bool rxFail;
    void (*_rs485tx) (void);
    void (*_rs485rx) (void);

} COM_TPort;

extern void COM_Select(uint8_t n);

extern void COM_Init(uint8_t n,
		UART_HandleTypeDef *huartPtr,
		void *rxBufPtr,
		uint16_t rxBufSize,
		uint16_t timeOut
		);
extern void COM_SetCallbacks485(void (*f485tx)(void), void (*f485rx)(void));

extern void COM_Write(void *pData, uint16_t Size);
extern void COM_WriteFast(void *pData, uint16_t Size);

extern bool COM_ReadByteFast(uint8_t *b);
extern bool COM_ReadByte(uint8_t *b);

extern bool COM_ReadFast(uint8_t *destBuffer, uint16_t needLen);
extern bool COM_Read(uint8_t *destBuffer, uint16_t needLen);

void COM_CleanRxFail(void);
bool COM_RxFail(void);

#endif /* UART_H_ */
