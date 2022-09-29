#ifndef EEPROM
#define EEPROM
#include "EEPROM.h"
#include "main.h"


typedef enum{
	SUCESS,
	FAIL = 1,
	TIMEOUT=2,
	BUSY=3
}ORET_STATUS;
 

#define EEPROM_CS_HIGH();   HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET);
#define EEPROM_CS_LOW();    HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET);

#define EEPROM_WREN  					0x06  /* Write Enable */	0000 0110
#define EEPROM_WRDI  					0x04  /* Write Disable */
#define EEPROM_RDSR  					0x05  /* Read Status Register */
#define EEPROM_WRSR  					0x01  /* Write Status Register */
#define EEPROM_READ  					0x03  /* Read from Memory Array */
#define EEPROM_WRITE 					0x02  /* Write to Memory Array */

#define EEPROM_WIP_FLAG       0x01  /* Write-In progress flag */

#define STARTMEM 		0x000
#define ENDMEM			0xFFF

ORET_STATUS Init_EEPROM(SPI_HandleTypeDef *hspi);
ORET_STATUS WriteByte();
ORET_STATUS WritePage(uint8_t* pTxBuffer, uint16_t pAddr, uint16_t NumOfBytes);
ORET_STATUS WriteBlock();
uint8_t ReadByte();
uint8_t ReadPage(uint8_t* pRxBuffer, uint16_t rAddr, uint16_t NumOfBytes);
uint8_t ReadBlock();
void WriteEnable();
void WriteDisable();
uint8_t ReadStatus();
uint8_t EEPROM_SPI_WaitStandbyState(void);
void EEPROM_SendInstruction(uint8_t *instruction, uint8_t size);
#endif