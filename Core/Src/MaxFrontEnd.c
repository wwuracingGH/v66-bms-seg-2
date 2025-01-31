 /*
 * MaxFrontEnd.h - Contains functionality for interfacing with the MAX
 *
 *  2022 Peter Schinske 	-	V63
 *  2023 Brandon Ramirez	-	V64
 *  2023 Chandler Johnston	-	V64
 */

#include "stm32g0xx_hal.h"
#include "MaxFrontEnd.h"
#include "stdio.h"

/*****************************************************************************************
 * Module private variables
 ****************************************************************************************/
static SPI_HandleTypeDef *maxspi;
static ADC_HandleTypeDef *maxadc;
static TIM_HandleTypeDef *maxtim;

// SPI Transmission Buffer
static uint8_t maxTxBuffer[] = { 0x00, 0x00, 0x00 };
// Max Status register
static uint8_t maxRxBuffer[] = { 0x00, 0x00, 0x00 };
// MAX SPI control buffer pointers
static uint8_t *balanceLower = &maxTxBuffer[0];
static uint8_t *balanceUpper = &maxTxBuffer[1];
static uint8_t *config = &maxTxBuffer[2];
// Voltage Data
static uint16_t sample_voltages[NUM_CELLS] = { 0 };
static uint16_t sample_voltages_avg = 0;

/*
 * Private Function Prototypes
 */
static uint8_t selectCell(uint8_t cellNum);
static void maxCalibrate(void);
static void delay_us(uint16_t us);

/*****************************************************************************************
 * Public Function Definitions
 ****************************************************************************************/
/*****************************************************************************************
* MaxInit() - PUBLIC
*   parameters: SPI, ADC, and Hardware timer peripheral pointers
*   return: none
*   description: Save peripheral memory locations and reset config TxBuffer
*****************************************************************************************/
void MaxInit(SPI_HandleTypeDef *hspi, ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim) {
	maxspi = hspi;
	maxadc = hadc;
	maxtim = htim;
	*balanceLower = 0x00;
	*balanceUpper = 0x00;
}

/*****************************************************************************************
* MaxDischargeCell() - PUBLIC
*   parameters: Cell index
*   return: none
*   description: Enables the discharge circuit for the selected cell
*****************************************************************************************/
void MaxDischargeCell(uint8_t cell) {
	*balanceLower = 0x01 << (cell);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_Transmit(maxspi, maxTxBuffer, BYTE_COUNT, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}

/*****************************************************************************************
* MaxDischargeTest() - PUBLIC
*   parameters: none
*   return: none
*   description: Cycle through discharging cells to test circuit and LEDs
*****************************************************************************************/
void MaxDischargeTest(void) {
	*config = 0x00;
	// Sequential
	for (int i = 0; i < NUM_CELLS; i++) {
		*balanceLower = 0x01 << i;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_SPI_Transmit(maxspi, maxTxBuffer, BYTE_COUNT, SPI_TIMEOUT);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_Delay(200);
	}
	for (int i = 0; i < NUM_SWITCHES; i++) {
		// Evens
		*balanceLower = 0b10101010;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_SPI_Transmit(maxspi, maxTxBuffer, BYTE_COUNT, SPI_TIMEOUT);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_Delay(200);

		// Odds
		*balanceLower = 0b01010101;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_SPI_Transmit(maxspi, maxTxBuffer, BYTE_COUNT, SPI_TIMEOUT);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_Delay(200);
	}

	*balanceLower = 0x00;
}

/*****************************************************************************************
* ADC_Select_CH7() - PRIVATE
*   parameters: none
*   return: none
*   description: Switches CH7 to active ADC channel
*****************************************************************************************/
void ADC_Select_CH7(void) {
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(maxadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

/*****************************************************************************************
* MaxSampleCharges() - PUBLIC
*   parameters: none
*   return: none
*   description: Takes MAX though sampling and hold states, read ADC values of cells
*****************************************************************************************/
void MaxSampleCharges(void) {
	//Empty transmission buffer and start sample phase
	*config = 0x00;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(maxspi, maxTxBuffer, maxRxBuffer, BYTE_COUNT, SPI_TIMEOUT);

	//wait for sample phase to complete
	HAL_Delay(T_SAMPLE);

	//start hold phase
	*config = SMPLB_HIGH;
	HAL_SPI_TransmitReceive(maxspi, maxTxBuffer, maxRxBuffer, BYTE_COUNT, SPI_TIMEOUT);

	//wait for sample cap voltages to shift to ground reference, at least 50.5 us
	delay_us(T_HOLD + T_LS_DELAY);

	// Switch ADC channels and start
	ADC_Select_CH7();

	//Measure voltage of every set of cells
	for (int i = 0; i < NUM_CELLS; i++) {

		// Reset config register to only ECS
		*config = ECS_HIGH | SMPLB_HIGH;

		// Select desired cell
		*config |= selectCell(i);

		//tell MAX14920 to measure voltage of cell i. Pulling SPI bus high triggers event
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(maxspi, maxTxBuffer, maxRxBuffer, BYTE_COUNT, SPI_TIMEOUT);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

		//Time delay to allow voltage measurement to settle.
		//According to MAX14920 datasheet, we should have a delay of over 5us.
		//Reading from ADC takes a few microseconds anyways.
		delay_us(T_SET);

		sample_voltages_avg = 0;
		for (int j = 0; j < BATTERY_SAMPLES; j++) {
			//read voltage of cell i from ADC
			HAL_ADC_Start(maxadc);
			HAL_ADC_PollForConversion(maxadc, HAL_MAX_DELAY); //HAL_MAX_DELAY replaces ADC_TIMEOUT
			sample_voltages_avg += HAL_ADC_GetValue(maxadc);
		}
		sample_voltages[i] = sample_voltages_avg / BATTERY_SAMPLES;

	}
	// Stop ADC
	HAL_ADC_Stop(maxadc);

	//end hold phase
	*config &= ~SMPLB_HIGH;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(maxspi, maxTxBuffer, maxRxBuffer, BYTE_COUNT, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}

/*****************************************************************************************
* MaxGetCellVoltages() - PUBLIC
*   parameters: Array to fill
*   return: none
*   description: Takes pointer to array and fills it with current cell voltage values
*****************************************************************************************/
void MAXGetCellVoltages(float *cell_voltages) {
	for(uint8_t i = 0; i < NUM_CELLS; i++) {
		cell_voltages[i] = 2 * (3.3 * (sample_voltages[i])/4096);
	}
}

/*****************************************************************************************
* delay_us() - PRIVATE
*   parameters: milliseconds to delay
*   return: none
*   description: Delays a number of milliseconds using a hardware timer
*****************************************************************************************/
static void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(maxtim,0);
	while (__HAL_TIM_GET_COUNTER(maxtim) < us);
}

/*****************************************************************************************
* selectCell() - PRIVATE
*   parameters: Cell index
*   return: none
*   description: Enables the discharge circuit for the selected cell
*****************************************************************************************/
uint8_t selectCell(uint8_t cellNum) {
	uint8_t bit0 = ((cellNum>>3) &0x01);
	uint8_t bit1 = ((cellNum>>1) &0x02);
	uint8_t bit2 = ((cellNum<<1) &0x04);
	uint8_t bit3 = ((cellNum<<3) &0x08);
	uint8_t selection = bit3|bit2|bit1|bit0;

	/*
	 * 7 = 0b0111
	 * bit0 = 0b0000
	 * bit1 = 0b0010
	 * bit2 = 0b0100
	 * bit3 = 0b1000
	 * */
	return selection<<3;
}

/*****************************************************************************************
* maxCalibrate() - PRIVATE
*   parameters: none
*   return: none
*   description: Takes MAX through calibration sequence
*****************************************************************************************/
void maxCalibrate(void) {
	// Run through calibration sequence
	*config = selectCell(1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(maxspi, maxTxBuffer, maxRxBuffer, BYTE_COUNT, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(10);

}
