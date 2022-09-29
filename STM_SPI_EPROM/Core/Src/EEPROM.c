#include "EEPROM.h"

static SPI_HandleTypeDef *EEPROM_SPI;

ORET_STATUS Init_EEPROM(SPI_HandleTypeDef *hspi){
	EEPROM_SPI =  &hspi2;
	HAL_GPIO_WritePin(WP_GPIO_Port,WP_Pin,1);
	HAL_GPIO_WritePin(HOLD_GPIO_Port,HOLD_Pin,1);
}
ORET_STATUS WriteByte(){
	
	
}
ORET_STATUS WritePage(uint8_t* pTxBuffer, uint16_t pAddr, uint16_t NumOfBytes){
	while (EEPROM_SPI->State != HAL_SPI_STATE_READY) {
        HAL_Delay(1);
    }
	HAL_StatusTypeDef EEPROM_STATUS = HAL_TIMEOUT;
	uint8_t command[3];
	WriteEnable();
	command[0] = EEPROM_WRITE;
	command[1] = pAddr >> 8;
	command[2] = pAddr;
		
		EEPROM_CS_LOW();
		
		EEPROM_SendInstruction(command, 3);
		do
		{
			EEPROM_STATUS = HAL_SPI_Transmit(EEPROM_SPI,pTxBuffer,NumOfBytes,100);
		}while(EEPROM_STATUS==HAL_BUSY||EEPROM_STATUS==HAL_ERROR||EEPROM_STATUS==HAL_TIMEOUT);
			HAL_UART_Transmit(&hlpuart1,"\n\rWrite Data:",sizeof("\n\rWrite Data:"),100);
		HAL_UART_Transmit(&hlpuart1,pTxBuffer,NumOfBytes,100);
		
//		while(EEPROM_STATUS==HAL_BUSY||EEPROM_STATUS==HAL_ERROR||EEPROM_STATUS==HAL_TIMEOUT){
//			
//			debug_printf("EEPROM Stat is  %d",EEPROM_STATUS);
//			DEBUG_LOG("Stuck");
//		}
		EEPROM_CS_HIGH();
		EEPROM_SPI_WaitStandbyState();
//		WriteDisable();
		 
		if(EEPROM_STATUS == HAL_ERROR){
			DEBUG_LOG("Error");
			return FAIL;
		}
		else{
			return SUCESS;
		}
	
}
ORET_STATUS WriteBlock(){
	
	
}
uint8_t ReadByte(){
	
	
}
uint8_t ReadPage(uint8_t* pRxBuffer, uint16_t rAddr, uint16_t NumOfBytes){
	
	while (EEPROM_SPI->State != HAL_SPI_STATE_READY) {
        HAL_Delay(1);
		DEBUG_LOG("SPI is busy"); 
    }
	HAL_StatusTypeDef EEPROM_STATUS;
	uint8_t command[3];
	
	command[0] = EEPROM_READ;
	command[1] = rAddr >> 8;  //0x01 00;  : 0x01
	command[2] = rAddr;
	
		EEPROM_CS_LOW();
	
	EEPROM_SendInstruction(command,3);
	
   while (HAL_SPI_Receive(EEPROM_SPI, pRxBuffer, NumOfBytes, 100) == HAL_BUSY) {
        HAL_Delay(1);
    };
	
		EEPROM_CS_HIGH();

	
	return SUCESS;

	
}
uint8_t ReadBlock(){
	
	
}
void WriteEnable(){
	
	//uint8_t command[1] = {EEPROM_WREN};
	uint8_t command[1] = {0x06};
		EEPROM_CS_LOW();
	EEPROM_SendInstruction((uint8_t*)command,1);
		EEPROM_CS_HIGH();
	
}
void WriteDisable(){
	
	uint8_t command[1] = {EEPROM_WRDI};
	EEPROM_CS_HIGH();
	EEPROM_SendInstruction((uint8_t*)command,1);
}
uint8_t ReadStatus(){
	
	
}

uint8_t EEPROM_SPI_WaitStandbyState() {
    uint8_t EEstatus[1] = { 0x00 };
    uint8_t command[1] = { EEPROM_RDSR };
		
		EEPROM_CS_LOW();
    EEPROM_SendInstruction((uint8_t*)command, 1);
    do {
        while (HAL_SPI_Receive(EEPROM_SPI, (uint8_t*)EEstatus, 1, 200) == HAL_BUSY) {
            HAL_Delay(1);
        };

        HAL_Delay(1);

    } while ((EEstatus[0] & EEPROM_WIP_FLAG) == SET); 
		EEPROM_CS_HIGH();
    return 0;
}

void EEPROM_SendInstruction(uint8_t *instruction, uint8_t size) {
    while (EEPROM_SPI->State == HAL_SPI_STATE_RESET) {
        HAL_Delay(1);
    }

    if (HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)instruction, (uint16_t)size, 100) != HAL_OK) {
        Error_Handler();
    }
}