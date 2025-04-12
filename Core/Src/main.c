/* USER CODE BEGIN Header */
/**
 * Segment controller to handle states and functionality of the BMS segment boards.
 *
 * Controller continuously cycles through actions based on state. in theory
 * Since no preemptive kernel is used, it is recommended to hold copies of data in main and
 * make sure copies are updated as fast as possible. Modules may be sampling data when interrupt occurs
 *
 *  2022 Peter Schinske 	-	V63
 *  2023 Brandon Ramirez	-	V64
 *  2023 Chandler Johnston	-	V64
 *  2024 Melana Evans		-	V66
 *  2024/5 Nicole Swierstra     -       V66/V67 :)
 */

#include "main.h"
#include "MaxFrontEnd.h"
#include "TempMonitor.h"

#define CELL_BALANCING 0


/* WARNING!:
 * Only enable this for segments that need it
 */
#define NUM_IGNORED_CELLS 5
#define IGNORED_CELLS_MASK 0b00000011111


/* TODO: this can be removed i am 200% sure */
#define BOARD_ID 0

/* If cell temps are outside this range they are discarded */
#define HIGHEST_CT 102.4f
#define LOWEST_CT  0.0f

#define MODE_NORMAL 	0
#define MODE_CHARGING 	1

/*
 * SPI data
 */
struct _SPI_Message {
	uint16_t highestVoltage;
	uint16_t avgVoltage;
	uint16_t lowestVoltage;
	uint16_t highestTemp;
	uint16_t avgTemp;
	uint16_t lowestTemp;
} SPI_Message = { 0,0,0,0,0,0};

struct _SPI_Control {
	uint16_t mode;
	uint16_t lowestVoltage;
	uint8_t _RESERVED[sizeof(SPI_Message) - 4];
} SPI_Control = {0, 0, {0}};

/*
 * Processing vars
 */
float cell_voltages[NUM_CELLS];
float sumVoltage;
float sumTemp;
uint8_t errCounter;

ADC_HandleTypeDef hadc1; 	/* ADC for cells and thermistors */
SPI_HandleTypeDef hspi1; 	/* SPI for MAX chips */
SPI_HandleTypeDef hspi2; 	/* SPI for peripheral communication */
TIM_HandleTypeDef htim14;	/* Used for the world's silliest delay function */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM14_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void Error_Handler(void);

void readData();
void transmitSPI();


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_TIM14_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();


	HAL_ADCEx_Calibration_Start(&hadc1);	/* Calibrate ADC */
	MaxInit(&hspi1,&hadc1,&htim14);		/* Pass peripheral pointers to MAX module and init */
	TMInit(&hadc1);				/* Pass ADC to temperature monitor and init */
	HAL_TIM_Base_Start(&htim14);		/* Start hardware timer */

	while(1) {
		readData();
		switch(SPI_Control.mode){
			case MODE_CHARGING:
#if CELL_BALANCING == 1
				MaxDischargeToVoltage(SPI_Control.lowestVoltage);
#endif
			break;

			case MODE_NORMAL:
				MaxStopDischarging(); /*I don't care enough to do this just once*/
			break;
		}
	}
}

void readData(){
	/* Sample & return cell voltages */
	MaxSampleCharges();
	MAXGetCellVoltages(cell_voltages);

	/* Calculate lowest, highest, and average cell voltages*/
	SPI_Message.lowestVoltage = 65535;
	SPI_Message.highestVoltage = 0;
	sumVoltage = 0;
	for (int i = 0; i < NUM_CELLS; i++) {
		if(NUM_IGNORED_CELLS > 0 && ((IGNORED_CELLS_MASK >> i) & 1)) continue;
		uint16_t voltInt = (uint16_t)(1000 * cell_voltages[i]);
		if (voltInt > SPI_Message.highestVoltage)
			SPI_Message.highestVoltage = voltInt;
		else if (voltInt < SPI_Message.lowestVoltage)
			SPI_Message.lowestVoltage = voltInt;
		    
		sumVoltage += cell_voltages[i];
	}
	SPI_Message.avgVoltage = (uint16_t) (1000 * (sumVoltage / ((NUM_CELLS) - (NUM_IGNORED_CELLS))));


	TMSampleTemps();

	SPI_Message.lowestTemp = 65535;
	SPI_Message.highestTemp = 0;
	sumTemp = 0;
	errCounter = 0;
	for (int i = 0; i < NUMTHERMISTORS; i++) {
		if (cellTemps[i] <= LOWEST_CT || cellTemps[i] > HIGHEST_CT){
			errCounter++;
		        continue;
		}

		uint16_t tempInt = (uint16_t)(10 * cellTemps[i]);

		if (tempInt > SPI_Message.highestTemp)
			SPI_Message.highestTemp = tempInt;
		else if (tempInt < SPI_Message.lowestTemp)
			SPI_Message.lowestTemp = tempInt;

		sumTemp += cellTemps[i];
	}

	/* TODO: figure out how to get rid of both of these */
	/* If we don't have any readings that set this value, change it to zero to prevent over-reads */
	if (SPI_Message.lowestTemp == 65535)
		SPI_Message.lowestTemp = 0;

	/* TODO: sum temp should always be more than 0 */
	if (sumTemp > 0)
		SPI_Message.avgTemp = (uint16_t) (10 * (sumTemp / ((NUMTHERMISTORS) - errCounter)));
	else
		SPI_Message.avgTemp = 0;

}

void transmitSPI(){
	SPI2->CR1 &= ~SPI_CR1_SSI; /* Turn on internal spi chip select */
	GPIOB->OTYPER &= ~(1 << 2); /* Turn on output for the MISO pin basically */

	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&SPI_Message, (uint8_t *)&SPI_Control, sizeof(SPI_Message), 1024);

	GPIOB->OTYPER |= (1 << 2); /* Turn off output for the MISO pin*/
	SPI2->CR1 |= SPI_CR1_SSI; /* Turn off internal spi chip select */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.LowPowerAutoPowerOff = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
    hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
    hadc1.Init.OversamplingMode = DISABLE;
    hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_SLAVE;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 7;
    hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK){
        Error_Handler();
    }

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void) {
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 16-1;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 65535;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_IC_Init(&htim14) != HAL_OK) {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim14, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIMEx_TISelection(&htim14, TIM_TIM14_TI1_MCO, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}

void EXTI4_15_IRQHandler(){
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9)){
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
		transmitSPI();
	}
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

	/* Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

	/* Configure GPIO pin : PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* Configure GPIO pins : PA1 PA2 PA3 PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	/* Software Chip select */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
