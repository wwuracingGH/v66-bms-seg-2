/*
 * 	BluetoothSPI.h
 *
 *  Created on: Jan 2, 2023
 *  Author: Chandler Johnston - V64
 */

#ifndef SRC_BLUETOOTHSPI_H_
#define SRC_BLUETOOTHSPI_H_

/*****************************************************************************************
* SPI_EV_IRQHandler() - PUBLIC
*   parameters: none
*   return: none
*   description: Is called on interrupt from bluetooth
*****************************************************************************************/
void SPI_EV_IRQHandler(SPI_HandleTypeDef *hspi);

/*****************************************************************************************
* BLESpiInit() - PUBLIC
*   parameters: Pointer to SPI peripheral
*   return: none
*   description: Initializes SPI peripheral
*****************************************************************************************/
void BLESpiInit(SPI_HandleTypeDef *hspi);

#endif /* SRC_BLUETOOTHSPI_H_ */
