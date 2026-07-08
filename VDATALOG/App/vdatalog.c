/**
 ******************************************************************************
 * @file    vdatalog.c
 * @brief   Platform independent application code for Vanilla Datalog
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "vdatalog.h"
//#include "App_model_Ism6hg256x_Mlc.h"
#include "custom_motion_sensors.h"
#include "custom_motion_sensors_ex.h"
#include "Ism6hg256x_reg.h"
#include "main.h"
#include "stdlib.h"
#include "simple_serial_tl.h"
#include "PnPL_init.h"
#include "timestamp.h"
//#include "App_model.h"
#include "Ism6hg256x_acc.h"
#include "Ism6hg256x_gyro.h"
#include "Ism6hg256x_mlc_app.h"
//#include "flash_memory.h"
#include "Ism6hg256x_mlc.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
extern bool is_MLC_configuration_in_flash(void);
extern void MLC_init_flash_memory(void);

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t led1_start_time 	= 0;
static uint32_t led10_start_time  	= 0;
uint32_t 		led10_timer  		= LED10_SLOW_TIMER;

/* Communication protocol */
SimpleSerialTL_t *pSstl;
volatile double ACCTimeStamp;
volatile double GYROTimeStamp;

volatile bool command_received;                        /* Flag when command is received from USB. */
uint32_t command_buffer_size;                         /* Size of the command received from USB. */
char command_buffer_static[APP_RX_DATA_SIZE];         /* Buffer containing the data received from USB. */
char *command_buffer_ptr;                              /* Pointer to the beginning of the command buffer. */
char *command_buffer_write_ptr;                        /* Pointer to the current write position in the command buffer. */


#if VD_DUMMY_DATA == 1
static int16_t dummy_counter = 0;
#endif
#if (VD_DUMMY_DATA == 1)
#if (VD_COMBO_SENSOR == 0)
static void InjectDummyData(uint8_t *buffer1, uint32_t data_size);
#else
static void InjectDummyData(uint8_t *buffer1, uint8_t *buffer2, uint32_t data_size);
#endif
#endif

#ifndef NO_PRINTF_USB
/* printf redirection over USB CDC */
int _write(int file, char *ptr, int len)
{
    CDC_Transmit_FS((uint8_t*)ptr, len);
    return len;
}
#endif

/**
 * @brief  Main function for Vanilla datalog application
 *         This function will never return
 * @param  handle: pointer to the HW interface for sensors
 */
void vdatalog_init(void *handle)
{
  /* Timestamp Timer initialization */
  TS_TIM_VD_Init();
  TS_TIM_MLC_Init();

  /* Serial Protocol (SSTL) initialization
   * Incoming configuration packets (PnPL) are managed in ProtocolScheduler(void)
   **/
  pSstl = SSTL_Alloc();
  SSTL_Init(pSstl);

  /* Frame format on UART
   * Max length of Datalog Payload buffer is defined by SSTL_MAX_ASYNC_PAYLOAD macro
   * and it depends on the ASPEP capabilities.
   * +--------------+-------------+------------------------------------+
   * | ASPEP Header | SSTL Header |       Datalog Payload buffer       |
   * +--------------+-------------+------------------------------------+
   * |    4 Bytes   |   4 Bytes   |               N Bytes              |
   * +--------------+-------------+------------------------------------+
   **/
 /* Initialize custom motion sensors abstraction layer */
  CUSTOM_MOTION_SENSOR_Init(SENSOR_0, MOTION_ACCELERO | MOTION_GYRO); /* ISM6HG256X */
  CUSTOM_MOTION_SENSOR_Init(SENSOR_1, MOTION_ACCELERO); /* IIS2DULPX */

 /* reset sensor registrs to dft values */
  MY_CUSTOM_MOTION_SENSOR_Reset (SENSOR_0);
  MY_CUSTOM_MOTION_SENSOR_Reset (SENSOR_1);   /* to be extended for IIS2DULPX */
 
  /* PnPL components initialization */
  PnPLSetAllocationFunctions(malloc, free);
  json_set_escape_slashes(0);
  PnPL_Components_Alloc();
  PnPL_Components_Init();

  /* Sensors FSMs initialization */
  FsmInit(&AccFsm,  FSM_STATE_DISABLED, AccTransitions,  sizeof_Acc_transitions,  A_G_INT2_EXTI_IRQn);    // Acc INT2
  FsmInit(&GyroFsm, FSM_STATE_DISABLED, GyroTransitions, sizeof_Gyro_transitions, A_G_INT2_EXTI_IRQn);    // Gyro INT2
  FsmInit(&MlcFsm,  FSM_STATE_DISABLED, MlcTransitions,  sizeof_Mlc_transitions,  A_G_INT1_EXTI_IRQn);     // MLC INT1

#if 0  
/* this code is to load MLC configuration from rom array at startup just for debugging */
#ifndef NO_FLASH_MEM  
  /* check if MLC configuration is already loaded in flash */
  if (!is_MLC_configuration_in_flash())
  {
    /* init in flash MLC magic number, fname and model file with default one */
    MLC_init_flash_memory();
  }
#endif
#endif

  /* set the default INT data ready mode, FIXME consider it can be changed at runtime by the MLC program */
  #ifdef DATA_READY_PULSED
  uint8_t Data;
  CUSTOM_MOTION_SENSOR_Read_Register(SENSOR_0, ISM6HG256X_CTRL4, &Data);
  Data = Data | 0x02; /* set drdy pulsed */
  CUSTOM_MOTION_SENSOR_Write_Register(SENSOR_0, ISM6HG256X_CTRL4, Data);
  #else
  uint8_t Data;
  CUSTOM_MOTION_SENSOR_Read_Register(SENSOR_0, ISM6HG256X_CTRL4, &Data);
  Data = Data & ~0x02; /* set drdy latched */
  CUSTOM_MOTION_SENSOR_Write_Register(SENSOR_0, ISM6HG256X_CTRL4, Data);
  #endif

	memset((char *) command_buffer_static, 0, APP_RX_DATA_SIZE);
 	command_buffer_ptr = &command_buffer_static[0];
	command_buffer_write_ptr = &command_buffer_static[0];
	command_buffer_size = 0;
	command_received = false;

#ifndef NO_PRINTF_USB 
  /* USB Device initialization */
	MX_USB_Device_Init();

	/* USB initialization timeout (required to stabilize the USB peripheral voltage and to open the terminal). */
	HAL_Delay(500);

        printf ("Vanilla Datalog initialized.\r\n"); // printf over USB CDC
#endif

// for debug purposes uncomment to enable by default acc, gyro or mlc
//      lsm6dsv16x_acc_set_enable(true);  // uncomment to enable ACC by default 
//      lsm6dsv16x_gyro_set_enable(false);  // uncomment to enable gyro by default
//      lsm6dsv16x_mlc_set_enable(true);  // uncomment to enable mlc by default      
//      load_lsm6dsv16x_mlc_configuration(lsm6dsv16x_mlc_conf_0, MEMS_CONF_ARRAY_LEN(lsm6dsv16x_mlc_conf_0)); // uncomment to load dft mlc program at startup
//      log_controller_start_log(0);  // uncomment start logging at startup by default
}

void vdatalog_main()
{
  /* Infinite loop */
  while(1)
  {
	/* Run the Protocol Scheduler to manage incoming messages */
    ProtocolScheduler();

    /* handle Start/Stop Log evts from PnPL cmds and from INT */
	  FsmHandleEvent(&AccFsm);
	  FsmHandleEvent(&GyroFsm);
    FsmHandleEvent(&MlcFsm);

#ifndef NO_PRINTF_USB
  /* dummy test of USB CDC COM sending back echo of each received data terminated with \CR\LF */
  	if (command_received) {         /* Check if command is received. */
 	  	command_received = false;
	  	printf("%s", command_buffer_ptr);               /* Send back on USB the command received. */
      printf ("\r\n");  /* send Carriage Return and Line Feed to terminate the line on terminal */
		  /* Resetting command buffer. */
		  command_buffer_size = command_buffer_write_ptr - command_buffer_ptr;    /* Size of the last command received. */
		  memset((char *) command_buffer_ptr, 0, command_buffer_size);            /* Resetting the command buffer. */
		  command_buffer_write_ptr = command_buffer_ptr;                          /* Move the write pointer to point to the initial position of the command buffer. */
		  command_buffer_size = 0;
	  }
#endif
    /* Go to sleep. The system wakes up on any IRQ (Systick as well) */
//    __WFI();
  }
}

/**
 * @brief  ProtocolScheduler
 *
 * In bare metal applications this function should be called periodically to check PnPL incoming msgs.
 * In an event driven application with a RTOS it could be invoked in a task when a new packet is received.
 */
void ProtocolScheduler(void)
{
  char *SerializedResponse = NULL;
  uint32_t rx_status;

  if(pSstl == NULL)
  {
    return;
  }
  /* Green LED flash once at GUI connection */
  if (HAL_GetTick() - led1_start_time >= LED1_TIMER)
  {
	  led1_start_time = 0;
//	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  }
  /* Blue LED blinking slow indicate prog is running */
  if (HAL_GetTick() - led10_start_time >= led10_timer)
  {
	  led10_start_time = HAL_GetTick();
      HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
  }

  rx_status = SSTL_ProcessRxFrame(pSstl, SSTL_NON_BLOCKING);
  if(rx_status == SSTL_NEW_PACKET_AVAILABLE)
  {
    uint8_t *p_rx_packet = NULL;
    uint32_t size = 0;
    if (SSTL_GetRxPacket(pSstl, &p_rx_packet, &size) == SSTL_OK)
    {
      /* PnPLCommand object declaration */
      PnPLCommand_t PnPLCommand;

      /* Parse received input message. (received_msg is the received input message to parse)
       * PnPLCommand object is filled by the PnPLParseCommand function.
       */
      PnPLParseCommand((char*) p_rx_packet, &PnPLCommand);

      if(PnPLCommand.comm_type == PNPL_CMD_SYSTEM_INFO)
      {
    	 led1_start_time = HAL_GetTick();
//         HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      }
      /* Check PnPLCommand type:
       * - If it is a GET Message, generate a response with complete or partial status
       * - PNPL_CMD_SYSTEM_INFO is for board identification only (FW_ID, BOARD_ID)
       */
      if(PnPLCommand.comm_type == PNPL_CMD_GET || PnPLCommand.comm_type == PNPL_CMD_SYSTEM_INFO)
      {
        /* Declare Serialized JSON response message and size */
        uint32_t size;

        /* Serialize message response */
        PnPLSerializeResponse(&PnPLCommand, &SerializedResponse, &size, 0);

        /* Set buffer that needs to be sent and its size */
        SSTL_SetResponse(pSstl, (uint8_t*) SerializedResponse, size);

        /* Start the transmission of the buffer that was previously set */
        if (SSTL_TxResponse(pSstl, SSTL_BLOCKING) != SSTL_PARTIAL_PACKET)
        {
          /* Free the json response unless the packet was segmented */
          pnpl_free(SerializedResponse);
          SerializedResponse = NULL;
        } 
      }
      else if(PnPLCommand.comm_type == PNPL_CMD_SET || PnPLCommand.comm_type == PNPL_CMD_COMMAND)
      {
        SSTL_TxAck(pSstl, SSTL_BLOCKING);
      }
    }
  }
  else if (rx_status == SSTL_ACK_RECEIVED) /* An ACK was received. */
  {
    /* Call again SSTL_SetResponse to send another frame of the segmented packet */
    if (SSTL_TxResponse(pSstl, SSTL_BLOCKING) != SSTL_PARTIAL_PACKET)
    {
      pnpl_free(SerializedResponse);
      SerializedResponse = NULL;
    }
  }
    else if (rx_status == SSTL_PARTIAL_PACKET)
  {
    // Send ACK to host for each partial packet received
    SSTL_TxAck(pSstl, SSTL_BLOCKING);
  }    
}

/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
#if defined(STM32U5) || defined(STM32H5) || defined(STM32U0)
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
#elif defined(STM32F4) || defined(STM32L4) || defined(STM32G4)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
#else
#warning "Add support for the current MCU family"
#endif
{
  switch(GPIO_Pin)
  {
/* INT1 carry the MLC and ODR triggered */
    case VD_SENSOR_INT1_PIN:
    {
//   	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
      /* INT1 can be from Acc/Gyro in ODR triggered */
      FsmSetEventFromINT(&AccFsm, FSM_EVT_INT);            
      FsmSetEventFromINT(&GyroFsm, FSM_EVT_INT);
      /* or from MLC in normal mode */
      FsmSetEventFromINT(&MlcFsm, FSM_EVT_INT);  

//      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);    
      break;
    }
/* INT2 carry the Acc and Gyro int or the PWM output configured as GPIO input in ODR triggered mode*/
    case VD_SENSOR_INT2_PIN:
    { 
//   	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);      
      FsmSetEventFromINT(&AccFsm, FSM_EVT_INT);            
 	    FsmSetEventFromINT(&GyroFsm, FSM_EVT_INT);      // Gyro INT on same pin
//    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);          
    	break;
    }
  }
}

void jump_to_bootloader( void )
{
  HAL_NVIC_DisableIRQ(A_G_INT1_EXTI_IRQn);
  HAL_NVIC_DisableIRQ(A_G_INT2_EXTI_IRQn);

  /* deinitialize peripherals */
#ifndef NO_PRINTF_USB   
  HAL_PCD_MspDeInit(&hpcd_USB_OTG_FS);
#endif
#ifndef NO_TIM3 
  HAL_TIM_Base_MspDeInit(&VD_TIMESTAMP_TIM);
#endif
#ifndef NO_TIM1
  HAL_TIM_Base_MspDeInit(&MLC_POLLING_TIM);
#endif
  HAL_UART_MspDeInit(&huart1);


	/*  Disable ICACHE */
	HAL_ICACHE_DeInit();

	/* Jump to user application */
	typedef  void (*pFunction)(void);
	pFunction JumpToApplication;
	uint32_t JumpAddress;
	JumpAddress = *(__IO uint32_t *) (0x0BF90000 + 4);
	JumpToApplication = (pFunction) JumpAddress;

	/* Initialize user application's Stack Pointer */
	__set_MSP(*(__IO uint32_t *) 0x0BF90000);
	JumpToApplication();

}

void ToggleFlashBank(void)
{
  FLASH_OBProgramInitTypeDef    OBInit = {0};
  OBInit.OptionType = OPTIONBYTE_USER;
	/* Set BFB2 bit to enable boot from Flash Bank2 */
	/* Allow Access to Flash control registers and user Flash */
	HAL_FLASH_Unlock();

	/* Allow Access to option bytes sector */
	HAL_FLASH_OB_Unlock();

	/* Get the Dual boot configuration status */
	HAL_FLASHEx_OBGetConfig(&OBInit);

	/* Enable/Disable dual boot feature */
	OBInit.OptionType = OPTIONBYTE_USER;
	OBInit.USERType   = OB_USER_SWAP_BANK;

	if (((OBInit.USERConfig) & (FLASH_OPTR_SWAP_BANK)) == FLASH_OPTR_SWAP_BANK)
	{
		OBInit.USERConfig &= ~FLASH_OPTR_SWAP_BANK;
	}
	else
	{
		OBInit.USERConfig = FLASH_OPTR_SWAP_BANK;
	}

	/* SYS_DEBUGF(SYS_DBG_LEVEL_WARNING, ("HW: Switching Bank\r\n")); */
	if(HAL_FLASHEx_OBProgram (&OBInit) != HAL_OK)
	{
		/*
	    Error occurred while setting option bytes configuration.
	    User can add here some code to deal with this error.
	    To know the code error, user can call function 'HAL_FLASH_GetError()'
		 */
		while(1);
	}

	/* Start the Option Bytes programming process */
	if (HAL_FLASH_OB_Launch() != HAL_OK) {
		/*
	    Error occurred while reloading option bytes configuration.
	    User can add here some code to deal with this error.
	    To know the code error, user can call function 'HAL_FLASH_GetError()'
		 */
		while(1);
	}
	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
}

#if (VD_DUMMY_DATA == 1)
#if (VD_COMBO_SENSOR == 0)
static void InjectDummyData(uint8_t *buffer1, uint32_t data_size)
#else
static void InjectDummyData(uint8_t *buffer1, uint8_t *buffer2, uint32_t data_size)
#endif
{
  uint32_t i;
  int16_t *p16_s1 = (int16_t*) buffer1;
  int16_t *p16_s2 = (int16_t*) buffer2;
  for(i = 0; i < data_size; i++)
  {
    /* Read sensor data */
#if (VD_COMBO_SENSOR == 0)
    *p16_s1++ = dummy_counter++;
    *p16_s1++ = dummy_counter++;
    *p16_s1++ = dummy_counter++;
#else
    *p16_s1++ = *p16_s2++ = dummy_counter++;
    *p16_s1++ = *p16_s2++ = dummy_counter++;
    *p16_s1++ = *p16_s2++ = dummy_counter++;
#endif
  }
}
#endif
  
