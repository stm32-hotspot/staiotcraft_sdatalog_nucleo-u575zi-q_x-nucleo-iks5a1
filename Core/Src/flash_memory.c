#include "flash_memory.h"
#include "stm32u5xx_hal.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32u5xx_hal_def.h"

/* Symbols to be defined in linker script (for all the compilers) */

extern const uint32_t            Flash_Bank1_start_address;  // symbol from linker script
extern const uint32_t            Flash_Bank1_end_address;    // symbol from linker script

extern const uint32_t            Flash_Bank2_start_address;  // symbol from linker script
extern const uint32_t            Flash_Bank2_end_address;    // symbol from linker script
 
/* Private function prototypes -----------------------------------------------*/
static uint32_t                 GetPage(uint32_t Address);
static uint32_t                 GetBank(uint32_t Address);

uint8_t * Write_To_Flash_From_Memory (uint8_t * memory_addr, const uint8_t * flash_addr_arg, uint32_t size)
{
uint32_t         FirstPage = 0, NbOfPages = 0, BankNumber = 0;
uint32_t         Address = 0, PageError = 0;
uint8_t        * buffer_ptr = NULL;  
uint8_t        * flash_addr = (uint8_t *)flash_addr_arg;
FLASH_EraseInitTypeDef EraseInitStruct;

  /* check arguments */
  if (memory_addr == NULL || flash_addr_arg == NULL || size == 0) return NULL;

  /* Round flash_addr up to next 8K (0x2000) page to align to next page */
  if ((uint32_t)flash_addr % 0x2000 != 0 )
  {
    flash_addr += 0x2000 - (uint32_t)flash_addr % 0x2000;
  }
  
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area, Get the 1st page to erase */
  FirstPage = GetPage((uint32_t)flash_addr);

  /* Get the number of pages to erase from 1st page */
  NbOfPages = GetPage((uint32_t)flash_addr + size) - FirstPage + 1;

  /* Get the bank */
  BankNumber = GetBank((uint32_t)flash_addr);

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = BankNumber;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    return NULL;
  }

  /* Program the user Flash area word by word
    (area defined by  flash_addr and size) ***********/
  
  Address = (uint32_t)flash_addr;
  buffer_ptr = memory_addr;
  
  while (Address < (uint32_t)flash_addr + size)  
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address, ((uint32_t)buffer_ptr)) == HAL_OK)
    {
      Address = Address + 16; /* increment flash addr to next quad word*/
      buffer_ptr = buffer_ptr + 16; /* increment RAM addr to next quad word*/
    }
    else
    {
      return NULL;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  /* Re-enable instruction cache */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    return NULL;
  }
  /* return the aligned flash mem address */
  return flash_addr;
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

static uint32_t GetBank(uint32_t Addr)
{
  if (Addr >= (uint32_t)&Flash_Bank1_start_address && Addr <= (uint32_t)&Flash_Bank1_end_address)  return FLASH_BANK_1;
  if (Addr >= (uint32_t)&Flash_Bank2_start_address && Addr <= (uint32_t)&Flash_Bank2_end_address)  return FLASH_BANK_2;
  return 0;
}


