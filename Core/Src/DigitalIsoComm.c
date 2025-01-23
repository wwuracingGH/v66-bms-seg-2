/*
 * DigitalIsoComm.c
 *
 * 2023	Chandler Johnston	-	V64
 *
 */

#include "stm32g0xx_hal.h"

/*
 * Private variables
 */
static SPI_HandleTypeDef *spi;

/*
 * Public Function Prototypes
 */

/*****************************************************************************************
* DigInit() - PUBLIC
*   parameters: Pointer to SPI peripheral
*   return: none
*   description: Initializes SPI peripheral
*****************************************************************************************/
void DigInit(SPI_HandleTypeDef *hspi) {
	spi = hspi;
}

