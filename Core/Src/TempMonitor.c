/*	TempMonitor.c - Contains functionality for reading temperature values from thermistors
 * 	Thermistors are rated for 100c maximum temp
 *
 *  2022 Brandon Ramirez	-	V63
 *  2023 Brandon Ramirez	- 	V64
 *  2023 Chandler Johnston	-	V64
 */

#include "stm32g0xx_hal.h"
#include "TempMonitor.h"
#include "math.h"

/*****************************************************************************************
 * Module private variables
 ****************************************************************************************/
static ADC_HandleTypeDef *tmadc;
static float cellTemps[NUMTHERMISTORS] = {0};	// Holds thermistor values in C

/* Lookup table
static const int16_t tmCelciusADCVal[22] = { 4026, 3771, 3516, 3261, 3006, 2751, 2568,
					     2394, 2202, 2014, 1848, 1690, 1496, 1345,
					     1200, 1074, 970,  866,  784,  694,  630, 0 };
*/
static float thermVoltage = 0;
static float thermResistance = 0;
static float tempInC = 0;
static float adcSampleAvg = 0;

/*
 * Private function prototypes
 */
void tmSelect(uint8_t index);
float tmConvertToTemp(float adcVal);
float tmConvertCtoF(float celcius);

/*****************************************************************************************
* TMInit() - PUBLIC
*   parameters: Pointer to ADC Peripheral
*   return: none
*   description: Stores memory location of ADC peripheral
*****************************************************************************************/
void TMInit(ADC_HandleTypeDef *hadc1){
	tmadc = hadc1;
}

/*****************************************************************************************
* TMSampleTemps() - PUBLIC
*   parameters: none
*   return: none
*   description: Cycles through all thermistors sampling their ADC values and converts to
*   celcius
*****************************************************************************************/
void TMSampleTemps() {
	float curtemp = 0;

	// Sample all ADC values and convert to C
	for(int i = 0; i < NUMTHERMISTORS; i++ ) {
		// Set ADCs to connect the desired thermistor
		tmSelect(i);

		// Right side thermistors
		ADC_Select_CH0();
		adcSampleAvg = 0;
		for (int j = 0; j < ADC_SAMPLES; j++) {
			HAL_ADC_Start(tmadc);
			HAL_ADC_PollForConversion(tmadc, TMADCTIMEOUT);
			adcSampleAvg = adcSampleAvg + HAL_ADC_GetValue(tmadc);
		}
		curtemp = adcSampleAvg / ADC_SAMPLES;
		HAL_ADC_Stop(tmadc);

		curtemp = tmConvertToTemp(curtemp);
		cellTemps[i] = curtemp;

	}

}

/*****************************************************************************************
* ADC_Select_CH0() - PUBLIC
*   parameters: none
*   return: none
*   description: Switches CH0 to active ADC channel
*****************************************************************************************/
void ADC_Select_CH0(void) {
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(tmadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

/*****************************************************************************************
* tmSelect() - PRIVATE
*   parameters: thermistor index
*   return: none
*   description: Selects given index on MUX
*****************************************************************************************/
void tmSelect(uint8_t index) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, index&0x01);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, index&0x02);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, index&0x04);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, index&0x08);
}

/*****************************************************************************************
* tmConvertToTemp() - PRIVATE
*   parameters: none
*   return: (float) Thermistor temperature in C
*   description: Converts given ADC value to temp based on calibration data from 3D printers
*   Linearly interpolates between measured ADC values at 5 degree (Celcius) intervals
*   Conversion to Fahrenheit is done later since all values were recorded in celcius during
*   calibration
*****************************************************************************************/
float tmConvertToTemp(float adcVal) {

	/* Convert thermistor reading to voltage */
	thermVoltage = VDD * (adcVal / 4096);

	/* Calculate resistance of thermistor based off voltage divider*/
	thermResistance = (thermVoltage * DIVIDER_RES)/(VDD - thermVoltage);

	/* The following equation is a re-arranged exponential equation y = ab^x => ln(y/a)/ln(b)*/
	/* a and b values in equation were determined through empirical testing and a regression of the data */
	/* Calculate celsius temp based off thermistor resistance */
	tempInC = logf(thermResistance/18970)/logf(0.9741);

	return tempInC;
}

