/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 *
 * Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "jsmn.h"
#include "string.h"
#include "nxjson.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId 		defaultTaskHandle;
osMessageQId 	msgQueueHandleATcmd;
osMessageQId 	msgQueueHandleLampcmd;
osMessageQId 	msgQueueHandleJSONParse;
osSemaphoreId   semHandleJSONMsgRead;

osSemaphoreDef(semJSONMsgRead);

/* USER CODE BEGIN Variables */
tmStruct 		CMDStruct;
rgbStruct 		colorStruct;
char error = 7;


osStatus osStatCode;

uint16_t 		resultCode;
jsmn_parser 	parser;
jsmntok_t 		tokens[128];
uint8_t 		jsonMsg[50];
uint8_t 		parseMsg[50];
uint8_t 		ATCmd[64];

extern uint8_t 	stringToRecieve[];

extern TIM_HandleTypeDef 	htim14;
extern TIM_HandleTypeDef 	htim16;
extern TIM_HandleTypeDef 	htim17;
extern UART_HandleTypeDef 	huart1;

char hundrNumbers[] = {0, 100, 200, 300, 400, 500, 600, 700, 800, 900};
char decNumbers[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90};

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

void TASK_USART_SEND(void const * argument)
{
	osEvent  	event;
	uint8_t 	paket = 0;
	uint8_t 	index = 0;
	semHandleJSONMsgRead = osSemaphoreCreate(osSemaphore(semJSONMsgRead), 1);
	while(1)
	{
		uint8_t semRelease = osSemaphoreWait(semHandleJSONMsgRead, 5);
		if(semRelease == osOK)
		{
			event = osMessageGet(msgQueueHandleLampcmd, osWaitForever);  // wait for message
			if (event.status == osEventMessage)
			{
				char code = event.value.v;
				if((code == '{') && (paket == 0))
				{
					paket = 1;
					memset(jsonMsg, 0, 50);
				}
				if((event.value.v == '\r') && (paket == 1))
				{
					paket = 0;
					osStatCode = osMessagePut(msgQueueHandleJSONParse, error, osWaitForever);
					index = 0;

				}
				if(paket == 1)
				{
					jsonMsg[index] = event.value.v;
					index++;
				}
			}
		}
		osSemaphoreRelease(semHandleJSONMsgRead);
		osDelay(1);
	}
}

void TASK_BL_FLASHING(void const * argument)
{
	while(1)
	{
		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, colorStruct.red);
		__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, colorStruct.green);
		__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, colorStruct.blue);
		osDelay(1);
	}
}

void TASK_JSON_PARSE(void const * argument)
{
	osEvent  event2;

	while(1)
	{
		event2 = osMessageGet(msgQueueHandleJSONParse, osWaitForever);
		if (event2.status == osEventMessage)
		{
			if(HAL_GPIO_ReadPin(TOUCH1_GPIO_Port, TOUCH1_Pin) == GPIO_PIN_SET)
			{
				int val = osSemaphoreWait(semHandleJSONMsgRead, osWaitForever);												// ждем освобождения семафора (дсотуп к ресурсу памяти)
				if(val == osOK)
				{
					memset(parseMsg, 0, 50);																				// семафор захвачен, делаем копирование памяти
					memcpy(parseMsg, jsonMsg, 50);
					jsmn_init(&parser);																					// парсим json сообщение
					resultCode = jsmn_parse(&parser, parseMsg, strlen(parseMsg), tokens, 128);
					jsonParseAnswerStructurisation(resultCode);
					colorParser();
					val = osSemaphoreRelease(semHandleJSONMsgRead);															// особождаем симафор
					if(val == osOK)
					{
						//HAL_UART_Transmit(&huart2, "release", 50, 500);
					}
					else
					{
						error = 10;
					}
				}

			}
		}

	}
}

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void)
{
	/* USER CODE BEGIN Init */
	HAL_UART_Receive_DMA(&huart1, stringToRecieve, RXBUFFERSIZE);
	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */


	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */


	/* USER CODE BEGIN RTOS_THREADS */
	osThreadDef(USART_SENDING, TASK_USART_SEND, osPriorityNormal, 0, 64);
	osThreadCreate(osThread(USART_SENDING), NULL);

	osThreadDef(BL_FLASHING, TASK_BL_FLASHING, osPriorityLow, 0, 32);
	osThreadCreate(osThread(BL_FLASHING), NULL);

	osThreadDef(JSON_PARSE, TASK_JSON_PARSE, osPriorityNormal, 0, 128);
	osThreadCreate(osThread(JSON_PARSE), NULL);
	/* USER CODE END RTOS_THREADS */

	/* Create the queue(s) */
	/* definition and creation of myQueue01 */
	osMessageQDef(qATcmd, 12, uint8_t);
	msgQueueHandleATcmd = osMessageCreate(osMessageQ(qATcmd), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	osMessageQDef(qLampcmd, 128, uint8_t);
	msgQueueHandleLampcmd = osMessageCreate(osMessageQ(qLampcmd), NULL);

	osMessageQDef(qJSONParser, 12, uint8_t);
	msgQueueHandleJSONParse = osMessageCreate(osMessageQ(qJSONParser), NULL);
	/* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */




void jsonParseAnswerStructurisation(int	quantityOfTokens)
{
		char keyString[50];
		char Prev_keyString[50];

		for (int i = 1; i < quantityOfTokens; i++) // resultCode == 0 => whole json string
		{
			jsmntok_t key = tokens[i];
			uint16_t length = key.end - key.start;

			if (length < 50)
			{
				memcpy(keyString, &parseMsg[key.start], length);
				keyString[length] = '\0';

				if (strcmp(Prev_keyString, "CMD") == 0)
					strcpy(CMDStruct.Cmd, keyString);
				else if (strcmp(Prev_keyString, "R") == 0)
					strcpy(CMDStruct.R, keyString);
				else if (strcmp(Prev_keyString, "G") == 0)
					strcpy(CMDStruct.G, keyString);
				else if (strcmp(Prev_keyString, "B") == 0)
					strcpy(CMDStruct.B, keyString);
				strcpy(Prev_keyString, keyString);
			}
		}
}



void colorParser()
{
	uint32_t *inter;
	inter = strchr(CMDStruct.R, '\0');
	{
		unsigned char index = (unsigned char*)inter-CMDStruct.R;
		if((index+1) == 4)
			colorStruct.red = hundrNumbers[CMDStruct.R[0]-48] + decNumbers[CMDStruct.R[1]-48] + (CMDStruct.R[2]-48);
		else if((index+1) == 3)
			colorStruct.red = decNumbers[CMDStruct.R[0]-48] + (CMDStruct.R[1]-48);
		else if((index +1) == 2)
			colorStruct.red = (CMDStruct.R[0]-48);

	}

	inter = strchr(CMDStruct.G, '\0');
	{
		unsigned char index = (unsigned char*)inter-CMDStruct.G;
		if((index+1) == 4)
			colorStruct.green = hundrNumbers[CMDStruct.G[0]-48] + decNumbers[CMDStruct.G[1]-48] + (CMDStruct.G[2]-48);
		else if((index+1) == 3)
			colorStruct.green = decNumbers[CMDStruct.G[0]-48] + (CMDStruct.G[1]-48);
		else if((index +1) == 2)
			colorStruct.green = (CMDStruct.G[0]-48);
	}

	inter = strchr(CMDStruct.B, '\0');
	{
		unsigned char index = (unsigned char*)inter-CMDStruct.B;
		if((index+1) == 4)
			colorStruct.blue = hundrNumbers[CMDStruct.B[0]-48] + decNumbers[CMDStruct.B[1]-48] + (CMDStruct.B[2]-48);
		else if((index+1) == 3)
			colorStruct.blue = decNumbers[CMDStruct.B[0]-48] + (CMDStruct.B[1]-48);
		else if((index +1) == 2)
			colorStruct.blue = (CMDStruct.B[0]-48);
	}

}
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
	error = 7;
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
