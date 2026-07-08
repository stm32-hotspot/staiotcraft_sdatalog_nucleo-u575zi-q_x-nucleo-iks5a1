/**
  ******************************************************************************
  * @file    flash_memory.h
  * @author  SRA Application Team
  * @brief   Flash memory management header file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_MEMORY_H
#define __FLASH_MEMORY_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
#define MLC_FILE_NAME_MAX_LEN           64
#define MLC_MAGIC_NUMBER_SIZE           4
//0x4D4C4321  // "MLC!" in ASCII
#define MLC_MAGIC_NUMBER                "MLC!"  
// symbols from linker script
extern const uint32_t                   MLC_magic_number_flash_address;            
#define MLC_MAGIC_NUMBER_FLASH_ADDRESS  &MLC_magic_number_flash_address

extern const uint8_t                    MLC_file_name_flash_address[MLC_FILE_NAME_MAX_LEN];   
#define MLC_FILE_NAME_FLASH_ADDRESS     &MLC_file_name_flash_address

extern const uint32_t                   MLC_program_num_lines; 
#define MLC_PROGRAM_NUM_LINES           &MLC_program_num_lines	
#define MLC_PROGRAM_NUM_LINES_SIZE      sizeof(MLC_program_num_lines)

extern const uint32_t                   MLC_program_flash_address; 
#define MLC_PROGRAM_FLASH_ADDRESS       &MLC_program_flash_address 		


#ifdef __cplusplus
}   // extern "C" 
#endif    

#endif /* __FLASH_MEMORY_H */