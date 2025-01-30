/*
 * TempMonitor.h
 *
 *  2022 Brandon Ramirez	-	V63
 *  2023 Brandon Ramirez	-	V64
 *  2023 Chandler Johnston	-	V64
 */

#ifndef SRC_TEMPMONITOR_H_
#define SRC_TEMPMONITOR_H_

/*****************************************************************************************
 * Definitions
 ****************************************************************************************/
#define NUMTHERMISTORS 10   	// Thermistors in single connector
#define TMADCTIMEOUT 100
#define VDD 3.3			// Supply voltage
#define DIVIDER_RES 5100	// Resistance (in Ohms) of upper voltage divider resistor
#define TABLETEMPINT 5
#define ADC_SAMPLES 16		// Number of samples averaged for reading

/*****************************************************************************************
* TMInit() - PUBLIC
*   parameters: Pointer to ADC Peripheral
*   return: none
*   description: Stores memory location of ADC peripheral
*****************************************************************************************/

static float cellTemps[NUMTHERMISTORS];

void TMInit(ADC_HandleTypeDef *hadc1);

/*****************************************************************************************
* TMSampleTemps() - PUBLIC
*   parameters: none
*   return: none
*   description: Cycles through all thermistors sampling their ADC values
*****************************************************************************************/
void TMSampleTemps(void);


/*****************************************************************************************
* ADC_Select_CH0() - PUBLIC
*   parameters: none
*   return: none
*   description: Switches CH0 to active ADC channel
*****************************************************************************************/
void ADC_Select_CH0(void);

#endif /* SRC_TEMPMONITOR_H_ */
