/*
 * Arduino port of libopeninv param_save
 * Uses STM32 HAL flash functions instead of libopencm3
 */

#include <Arduino.h>
#include "stm32f4xx_hal.h"
#include "params.h"
#include "param_save.h"
#include "my_string.h"

// Flash configuration - save to last flash page
#ifndef PARAM_BLKSIZE
#define PARAM_BLKSIZE 2048  // Flash page size for STM32F4
#endif

#ifndef PARAM_BLKNUM
#define PARAM_BLKNUM 1      // Use last flash page
#endif

#define NUM_PARAMS ((PARAM_BLKSIZE - 8) / sizeof(PARAM_ENTRY))
#define PARAM_WORDS (PARAM_BLKSIZE / 4)

typedef struct
{
   uint16_t key;
   uint8_t dummy;
   uint8_t flags;
   uint32_t value;
} PARAM_ENTRY;

typedef struct
{
   PARAM_ENTRY data[NUM_PARAMS];
   uint32_t crc;
   uint32_t padding;
} PARAM_PAGE;

// Simple CRC32 calculation
static uint32_t calculate_crc32(uint32_t *data, uint32_t length)
{
   uint32_t crc = 0xFFFFFFFF;

   for (uint32_t i = 0; i < length; i++)
   {
      crc ^= data[i];
      for (int j = 0; j < 32; j++)
      {
         if (crc & 1)
            crc = (crc >> 1) ^ 0xEDB88320;
         else
            crc = crc >> 1;
      }
   }

   return ~crc;
}

static uint32_t GetFlashAddress()
{
   // Get flash size from device signature
   uint16_t flashSize = *((uint16_t*)0x1FFF7A22); // Flash size in KB

   // Return address of last flash page
   return FLASH_BASE + (flashSize * 1024) - (PARAM_BLKNUM * PARAM_BLKSIZE);
}

static uint32_t GetFlashSector(uint32_t address)
{
   // Determine which flash sector the address belongs to
   // STM32F4 has variable sector sizes
   if (address < 0x08004000) return FLASH_SECTOR_0;
   if (address < 0x08008000) return FLASH_SECTOR_1;
   if (address < 0x0800C000) return FLASH_SECTOR_2;
   if (address < 0x08010000) return FLASH_SECTOR_3;
   if (address < 0x08020000) return FLASH_SECTOR_4;
   if (address < 0x08040000) return FLASH_SECTOR_5;
   if (address < 0x08060000) return FLASH_SECTOR_6;
   if (address < 0x08080000) return FLASH_SECTOR_7;
   #ifdef FLASH_SECTOR_8
   if (address < 0x080A0000) return FLASH_SECTOR_8;
   if (address < 0x080C0000) return FLASH_SECTOR_9;
   if (address < 0x080E0000) return FLASH_SECTOR_10;
   return FLASH_SECTOR_11;
   #else
   return FLASH_SECTOR_7;
   #endif
}

/**
* Save parameters to flash
*
* @return CRC of parameter flash page
*/
uint32_t parm_save()
{
   PARAM_PAGE parmPage;
   uint32_t idx;
   uint32_t paramAddress = GetFlashAddress();
   uint32_t sector = GetFlashSector(paramAddress);

   memset32((int*)&parmPage, 0xFFFFFFFF, PARAM_WORDS);

   // Copy parameter values and keys to block structure
   for (idx = 0; idx < NUM_PARAMS && idx < Param::PARAM_LAST; idx++)
   {
      if (Param::GetType((Param::PARAM_NUM)idx) == Param::TYPE_PARAM)
      {
         const Param::Attributes *pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
         parmPage.data[idx].flags = (uint8_t)Param::GetFlag((Param::PARAM_NUM)idx);
         parmPage.data[idx].key = pAtr->id;
         parmPage.data[idx].value = Param::Get((Param::PARAM_NUM)idx);
      }
   }

   parmPage.crc = calculate_crc32((uint32_t*)&parmPage, 2 * NUM_PARAMS);

   // Unlock flash
   HAL_FLASH_Unlock();

   // Erase sector
   FLASH_EraseInitTypeDef eraseInit;
   uint32_t sectorError;

   eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
   eraseInit.Sector = sector;
   eraseInit.NbSectors = 1;
   eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V to 3.6V

   HAL_FLASHEx_Erase(&eraseInit, &sectorError);

   // Program flash
   for (idx = 0; idx < PARAM_WORDS; idx++)
   {
      uint32_t* pData = ((uint32_t*)&parmPage) + idx;
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, paramAddress + idx * sizeof(uint32_t), *pData);
   }

   // Lock flash
   HAL_FLASH_Lock();

   return parmPage.crc;
}

/**
* Load parameters from flash
*
* @retval 0 Parameters loaded successfully
* @retval -1 CRC error, parameters not loaded
*/
int parm_load()
{
   uint32_t paramAddress = GetFlashAddress();
   PARAM_PAGE *parmPage = (PARAM_PAGE *)paramAddress;

   uint32_t crc = calculate_crc32((uint32_t*)parmPage, 2 * NUM_PARAMS);

   if (crc == parmPage->crc)
   {
      for (unsigned int idxPage = 0; idxPage < NUM_PARAMS; idxPage++)
      {
         Param::PARAM_NUM idx = Param::NumFromId(parmPage->data[idxPage].key);
         if (idx != Param::PARAM_INVALID && Param::GetType((Param::PARAM_NUM)idx) == Param::TYPE_PARAM)
         {
            Param::SetFixed(idx, parmPage->data[idxPage].value);
            Param::SetFlagsRaw(idx, parmPage->data[idxPage].flags);
         }
      }
      return 0;
   }

   return -1;
}
