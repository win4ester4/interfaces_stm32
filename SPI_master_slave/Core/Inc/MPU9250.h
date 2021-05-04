#ifndef MPU925_H_
#define MPU925_H_
#endif /* MPU925_H_ */

#include "stm32f4xx_hal.h"
#include "main.h"
extern SPI_HandleTypeDef hspi1;

#define CS_ON HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET)
#define CS_OFF HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET)

//-------------------------------------------

int writeRegister(uint8_t subAddress, uint8_t data);
int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
int writeAK8963Register(uint8_t subAddress, uint8_t data);
int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
int whoAmI();
int whoAmIAK8963();
void MPU_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
void MPU_SPI_Write (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
int setSrd(uint8_t srd);
int enableDataReadyInterrupt();
int MPU_begin();
int readSensor();
//-------------------------------------------

extern int16_t _axcounts,_aycounts,_azcounts;
extern int16_t _gxcounts,_gycounts,_gzcounts;
extern int16_t _hxcounts,_hycounts,_hzcounts;
extern int16_t _tcounts;
