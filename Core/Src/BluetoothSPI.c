/*
 * 	BluetoothSPI.h
 *
 *  Created on: Jan 2, 2023
 *  Author: Chandler Johnston - V64
 */

#include "stm32g0xx_hal.h"
#include "BluetoothSPI.h"

SPI_HandleTypeDef *btspi;

/*****************************************************************************************
* BLESpiInit() - PUBLIC
*   parameters: Pointer to SPI peripheral
*   return: none
*   description: Initializes SPI peripheral
*****************************************************************************************/
void BLESpiInit(SPI_HandleTypeDef *hspi){
	btspi = hspi;
}

/*****************************************************************************************
* SPI_EV_IRQHandler() - PUBLIC
*   parameters: none
*   return: none
*   description: Is called on interrupt from bluetooth
*****************************************************************************************/
void SPI_EV_IRQHandler(SPI_HandleTypeDef *hspi){

}
