/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

#include "udp_server.h"
#include "frame_stack.h"
#include "button_led.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern unsigned short sys_tmr;

extern CAN_HandleTypeDef hcan1;
static uint8_t				 can_frame[40];
static uint8_t				 ext_can_frame[40];
static uint8_t				 ext_can_frame_id[256][40];
static uint8_t               TxData[8];
static CAN_TxHeaderTypeDef   TxHeader;
static uint32_t              TxMailbox=0;
static CAN_RxHeaderTypeDef   RxHeader;
static uint8_t               RxData[8];

unsigned short gate_id = 0xFE;
unsigned char device_id = 0xFF;
unsigned char from_id = 0x00;


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId canTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void initCANFilter() {
	CAN_FilterTypeDef  sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

static void send_full_frame(uint8_t len, uint8_t *ptr) {
	uint8_t i=len;
	uint8_t cur_pckt = 1;
	uint8_t pckt_cnt = 0;

	while(i) {
		pckt_cnt++;
		if(i<=6) {i=0;}
		else i-=7;
	}
	i=1;

	while(len>0) {
		TxHeader.StdId = gate_id;
		TxHeader.ExtId = 0;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.TransmitGlobalTime = DISABLE;
		if(len<=6) { // last packet
			TxHeader.DLC = 2+len;
			TxData[0] = (cur_pckt&0x0F)|((pckt_cnt&0x0F)<<4); // current packet number and packets cnt
			TxData[1] = device_id;
			for(i=0;i<len;i++) TxData[i+2] = ptr[(cur_pckt-1)*7+i];
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {osDelay(1);}
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			cur_pckt++;
			len=0;
		}else {
			TxHeader.DLC = 0x08;
			TxData[0] = (cur_pckt&0x0F)|((pckt_cnt&0x0F)<<4); // current packet number and packets cnt
			for(i=0;i<7;i++) TxData[i+1] = ptr[(cur_pckt-1)*7+i];
			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {osDelay(1);}
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			cur_pckt++;
			len-=7;
		}
	}
	//HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin);
}

static void send_frame(uint8_t num, uint8_t *ptr) {
	static uint32_t i;
	TxHeader.StdId = gate_id;
	TxHeader.ExtId = 0;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;
	switch(num) {
	case 1:
		TxHeader.DLC = 0x08;
		TxData[0] = 0x01;
		for(i=0;i<7;i++) TxData[i+1] = ptr[i];
		//while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {osDelay(1);}
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		break;
	case 2:
		TxHeader.DLC = 0x08;
		TxData[0] = 0x02;
		for(i=0;i<7;i++) TxData[i+1] = ptr[i+7];
		//while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {osDelay(1);}
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		break;
	case 3:
		TxHeader.DLC = 0x08;
		TxData[0] = 0x03;
		for(i=0;i<6;i++) TxData[i+1] = ptr[i+14];
		TxData[7] = device_id;
		//while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {osDelay(1);}
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		break;
	}
}
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartCanTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of canTask */
  osThreadDef(canTask, StartCanTask, osPriorityIdle, 0, 512);
  canTaskHandle = osThreadCreate(osThread(canTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN StartDefaultTask */


  udp_server_init();

  /* Infinite loop */
  for(;;)
  {
	  //toggle_first_led(GREEN);
	  osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCanTask */
/**
* @brief Function implementing the canTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTask */
void StartCanTask(void const * argument)
{
  /* USER CODE BEGIN StartCanTask */
	static unsigned char cnt = 0;
	static unsigned char length = 0;

	initCANFilter();
	HAL_CAN_Start(&hcan1);
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
	  Error_Handler();
	}

	init_frames();

  /* Infinite loop */
  for(;;)
  {
	  if(sys_tmr>=20) {
		  sys_tmr = 0;
		  length = get_frame(can_frame);
		  if(length) {
			  send_full_frame(length,can_frame);

		  }else toggle_second_led(RED);
	  }
	  /*if(cnt) cnt--;
	  if(cnt==0) {
		  length = get_frame(can_frame);
		  if(length) {
			  cnt=20;
			  send_full_frame(length,can_frame);
			  //toggle_second_led(GREEN);
		  }//else toggle_second_led(RED);
	  }*/
	  osDelay(1);
  }
  /* USER CODE END StartCanTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint8_t i = 0;
	/*
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {

	  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		  if(RxData[0]==3 && RxHeader.DLC==8) toggle_second_led(GREEN);

		  if(RxData[0]==1) {for(i=0;i<7;i++) ext_can_frame[i]=RxData[i+1];}
		  else if(RxData[0]==2) {for(i=0;i<7;i++) ext_can_frame[i+7]=RxData[i+1];}
		  else if(RxData[0]==3) {
			  for(i=0;i<6;i++) ext_can_frame[i+14]=RxData[i+1];
			  add_can_frame(ext_can_frame);
			  from_id = RxHeader.StdId;
		  }
	  }
  }*/
	static uint8_t cur_num = 0;
	static uint8_t cnt = 0;
	static uint8_t length = 0;
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {

	  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

		  cur_num = RxData[0] & 0x0F;
		  cnt = RxData[0] >> 4;
		  if(cur_num==cnt) {
			  for(i=0;i<RxHeader.DLC-2;i++) ext_can_frame_id[RxHeader.StdId][(cur_num-1)*7+i]=RxData[i+2];
			  length = (cnt-1)*7+RxHeader.DLC-2;
			  for(i=0;i<length;i++) ext_can_frame[i]=ext_can_frame_id[RxHeader.StdId][i];
			  add_can_frame(ext_can_frame,length);
			  from_id = RxHeader.StdId;


		  }else {
			  for(i=0;i<7;i++) ext_can_frame_id[RxHeader.StdId][(cur_num-1)*7+i]=RxData[i+1];
		  }
	  }
  }
}
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
