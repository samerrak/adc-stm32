/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE 8
#define VREFCHARAC 3000 //ADC mV voltage from pg 34 in STM32L475xx manual
#define VREFINTCAL (*((uint16_t *) 0x1FFF75AA)) //read from memory 0x1FFF 75AA - 0x1FFF 75AB
// read from Functional overview STM32L4S5xx manual pg 44
#define TSCAL1 (*((uint16_t *) 0x1FFF75A8)) // @30C
#define TSCAL1TEMP 30
#define TSCAL2 (*((uint16_t *) 0x1FFF75CA)) // @130C
#define TSCAL2TEMP 130



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
uint32_t vref=0;
uint32_t temp=0; // set as global variables to watch them
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void MX_ADC1_Init(int mode); // change signature; 1 mode is Vref, 0 mode is Temp

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  MX_ADC1_Init(0);

  // first pass of program ensure that it is in vref mode to get value to calculate temperature
  vref=refVoltage();
  int mode = 0; // mode 0 vref, mode 1 temp
  HAL_GPIO_WritePin(myLed2_GPIO_Port, myLed2_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	int status;
	for (int i=0; i< 3; i++) { //polling sometimes not capturing so try twice
		status = HAL_GPIO_ReadPin(myButton_GPIO_Port, myButton_Pin); //tutorial
		if (status == 0) {
			break;
		}
	}

	if (status == 0) {
		if (mode == 0) {
			mode = 1; //pressed need to switch to temperature mode and turn on LED since it was in vref mode
			HAL_GPIO_WritePin(myLed2_GPIO_Port, myLed2_Pin, GPIO_PIN_SET);
			MX_ADC1_Init(1);

		}
		else {
			mode = 0; // pressed need to switch from temp mode to vref mode and turn off LED
			HAL_GPIO_WritePin(myLed2_GPIO_Port, myLed2_Pin, GPIO_PIN_RESET);
			MX_ADC1_Init(0);
		}
	}
	
 
  if (HAL_GPIO_ReadPin(myLed2_GPIO_Port, myLed2_Pin) == GPIO_PIN_SET) {
		temp = tempSensor(vref); // if LED is one keep on reading temperature
  }
  else {
		vref = refVoltage(); // if LED is off keep on reasing reference voltage

  }



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(int mode)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV10; // prescaler
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  if (mode == 0) {
	  sConfig.Channel = ADC_CHANNEL_VREFINT;
	  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5; //STM32L475xx manual pg 93, 12 us / (10/48Mhz) = 57.6 CCs
  }
  else {
	  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;//STM32L475xx manual pg 93, 120 us / (10/48Mhz) = 576 CCs
  }
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(myLed2_GPIO_Port, myLed2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : myButton_Pin */
  GPIO_InitStruct.Pin = myButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(myButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : myLed2_Pin */
  GPIO_InitStruct.Pin = myLed2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(myLed2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint32_t pollData() {
  // Begin Calibration
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // pg 142 in HAL driver manual

	// Start the conversion
	HAL_ADC_Start(&hadc1); // pg 109 in HAL Driver Manual

  // Poll for reference values
  if (HAL_OK != HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY-1))  //pg 69 in HAL driver manual // change time
    return -1; // if the return type is not HAL_OK, and is one of the other enums then we have failed
	uint32_t POLLEDDATA = HAL_ADC_GetValue(&hadc1); // pg 110 in HAL Driver Manual

  return POLLEDDATA;
}


int buttonOn() {
	int status = HAL_GPIO_ReadPin(myButton_GPIO_Port, myButton_Pin); //tutorial
	if (status == 0) {
      HAL_GPIO_WritePin(myLed2_GPIO_Port, myLed2_Pin, GPIO_PIN_SET);
	}
	else {
      HAL_GPIO_WritePin(myLed2_GPIO_Port, myLed2_Pin, GPIO_PIN_RESET);
	}

	return 0;
}

uint32_t refVoltage() {
	uint32_t VREFINTDATA = pollData();

	if (VREFINTDATA == -1)
		return -1;

	// Convert to real voltage value
	uint32_t VOLTAGEREF = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(VREFINTDATA, ADC_RESOLUTION_12B); //pg 129 in HAL Driver Manual
	// Our implementation pg 692 RM0432 Reference manual
	uint32_t VOLTAGEREF1 = VREFCHARAC * (VREFINTCAL + 0.0) / (VREFINTDATA); // I add 0.0 to convert to floats to have accurate result

	if (HAL_OK != HAL_ADC_Stop(&hadc1)) // pg 107 in HAL driver manual
    return -1;

	return ((VOLTAGEREF + VOLTAGEREF1) / 2);

}

// In note section the VREF+ value is needed because the TSDATA is under the assumption that is taken at 3V so an adjustment must be made
// pg 691 in note section in chip manual or RM0432 Reference manual
uint32_t tempSensor(uint32_t VREFPLUS) {
  uint32_t TSDATA = pollData();

   if (TSDATA == -1)
    return -1;

  //Convert to actual temperature value
  uint32_t TEMPREAL = __HAL_ADC_CALC_TEMPERATURE(VREFPLUS, TSDATA, ADC_RESOLUTION_12B); //pg 130 in HAL Driver Manual

  //Our implementation pg 691 RM0432 Reference manual, Based on the manual TSDATA must be converted to the VREF+
  uint32_t TEMPREAL1 = (((TSCAL2TEMP - TSCAL1TEMP + 0.0)/(TSCAL2 - TSCAL1)) * ((((3304+0.0)/3000)*TSDATA) - TSCAL1))+ 30;

  if (HAL_OK != HAL_ADC_Stop(&hadc1)) // pg 107 in HAL driver manual
    return -1;


  return (TEMPREAL + TEMPREAL1) / 2;

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
