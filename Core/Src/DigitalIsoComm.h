/*
 * DigitalIsoComm.c
 *
 * 2023	Chandler Johnston	-	V64
 *
 */

#ifndef SRC_DIGITALISOCOMM_H_
#define SRC_DIGITALISOCOMM_H_

/*****************************************************************************************
* DigInit() - PUBLIC
*   parameters: Pointer to SPI peripheral
*   return: none
*   description: Initializes SPI peripheral
*****************************************************************************************/
void DigInit(SPI_HandleTypeDef *hspi);

#endif /* SRC_DIGITALISOCOMM_H_ */
