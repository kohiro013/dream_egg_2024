
#include <string.h>
#include "module.h"

#define FLASH_BANK_NUM		FLASH_BANK_2
#define FLASH_SECTOR_NUM	FLASH_SECTOR_0
#define FLASH_START_ADRESS	(0x08100000)

/* ----------------------------------------------------------------------------------
	Flashのsector1を消去
-----------------------------------------------------------------------------------*/
uint8_t Flash_EraseData( void )
{
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Banks = FLASH_BANK_NUM;
	EraseInitStruct.Sector = FLASH_SECTOR_NUM;
	EraseInitStruct.NbSectors = 127;

	// Eraseに失敗したsector番号がerror_sectorに入る
	// 正常にEraseができたときは0xFFFFFFFFが入る
	uint32_t error_sector;
	HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

	HAL_FLASH_Lock();

	return result == HAL_OK && error_sector == 0xFFFFFFFF;
}

/* ----------------------------------------------------------------------------------
	Flashのsector1を読み出す
-----------------------------------------------------------------------------------*/
void Flash_ReadData( uint8_t *data, uint32_t size )
{
	memcpy( data, (uint8_t*)FLASH_START_ADRESS, size );
}

/* ----------------------------------------------------------------------------------
	Flashのsectorを消去後にデータを書き込む
-----------------------------------------------------------------------------------*/
uint8_t Flash_WriteData( uint16_t *data, size_t size  )
{
	size /= 16;

	if( !Flash_EraseData() ) return 0;
	HAL_FLASH_Unlock();

	uint32_t address = FLASH_START_ADRESS;
	HAL_StatusTypeDef result;
	do {
		uint8_t buffer[16];
		for(int8_t i = 0; i < 8; ++i) {
			buffer[i * 2] = (uint8_t)((*data) &0xff);
			buffer[i * 2 + 1] = (uint8_t)(((*data) >> 8) &0xff);
			printf("%02x%02x, ", buffer[i * 2], buffer[i * 2 + 1]);
		}
		printf("\r\n");

		result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address, (uint32_t)buffer);
		if( result != HAL_OK ) {
			break;
		} else {}
		address += sizeof(buffer);
		data += sizeof(buffer);
	} while( --size );
	printf("\r\n");

	HAL_FLASH_Lock();

	return result == HAL_OK;
}
