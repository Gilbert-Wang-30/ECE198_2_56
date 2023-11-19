/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motor1_on(TIM_HandleTypeDef *htim2){
	__HAL_TIM_SET_COMPARE(htim2,TIM_CHANNEL_1, 990);
}
void motor2_on(TIM_HandleTypeDef *htim2){
	__HAL_TIM_SET_COMPARE(htim2,TIM_CHANNEL_2, 1030);
}
void motor3_on(TIM_HandleTypeDef *htim2){
	__HAL_TIM_SET_COMPARE(htim2,TIM_CHANNEL_3, 1080);
}
void motor4_on(TIM_HandleTypeDef *htim3){
	__HAL_TIM_SET_COMPARE(htim3,TIM_CHANNEL_1, 1080);
}
void motor5_on(TIM_HandleTypeDef *htim3){
	__HAL_TIM_SET_COMPARE(htim3,TIM_CHANNEL_2, 1080);
}
void motor6_on(TIM_HandleTypeDef *htim3){
	__HAL_TIM_SET_COMPARE(htim3,TIM_CHANNEL_3, 1050);
}
void motor1_off(TIM_HandleTypeDef *htim2){
	__HAL_TIM_SET_COMPARE(htim2,TIM_CHANNEL_1, 780);
}
void motor2_off(TIM_HandleTypeDef *htim2){
	__HAL_TIM_SET_COMPARE(htim2,TIM_CHANNEL_2, 850);
}
void motor3_off(TIM_HandleTypeDef *htim2){
	__HAL_TIM_SET_COMPARE(htim2,TIM_CHANNEL_3, 1010);
}
void motor4_off(TIM_HandleTypeDef *htim3){
	__HAL_TIM_SET_COMPARE(htim3,TIM_CHANNEL_1, 1360);
}
void motor5_off(TIM_HandleTypeDef *htim3){
	__HAL_TIM_SET_COMPARE(htim3,TIM_CHANNEL_2, 1250);
}
void motor6_off(TIM_HandleTypeDef *htim3){
	__HAL_TIM_SET_COMPARE(htim3,TIM_CHANNEL_3, 1220);
}
void display(char c){
	if(c == 'a'){
		motor1_off(&htim2);
		motor2_off(&htim2);
		motor3_off(&htim2);
		motor4_on(&htim3);
		motor5_off(&htim3);
		motor6_off(&htim3);
	}else if(c == 'b'){
		motor1_off(&htim2);
		motor2_off(&htim2);
		motor3_off(&htim2);
		motor4_on(&htim3);
		motor5_on(&htim3);
		motor6_off(&htim3);
	}else if(c == 'c'){
		motor1_on(&htim2);
		motor2_off(&htim2);
		motor3_off(&htim2);
		motor4_on(&htim3);
		motor5_off(&htim3);
		motor6_off(&htim3);
	}else if(c == 'd'){
		motor1_on(&htim2);
		motor2_on(&htim2);
		motor3_off(&htim2);
		motor4_on(&htim3);
		motor5_off(&htim3);
		motor6_off(&htim3);
	}else if(c == 'e'){
		motor1_off(&htim2);
		motor2_on(&htim2);
		motor3_off(&htim2);
		motor4_on(&htim3);
		motor5_off(&htim3);
		motor6_off(&htim3);
	}else if(c == 'f'){
		motor1_on(&htim2);
		motor2_off(&htim2);
		motor3_off(&htim2);
		motor4_on(&htim3);
		motor5_on(&htim3);
		motor6_off(&htim3);
	}else if(c == 'g'){
		motor1_on(&htim2);
		motor2_on(&htim2);
		motor3_off(&htim2);
		motor4_on(&htim3);
		motor5_on(&htim3);
		motor6_off(&htim3);
	}else if(c == 'h'){
		motor1_off(&htim2);
		motor2_on(&htim2);
		motor3_off(&htim2);
		motor4_on(&htim3);
		motor5_on(&htim3);
		motor6_off(&htim3);
	}else if(c == 'i'){
		motor1_on(&htim2);
		motor2_off(&htim2);
		motor3_off(&htim2);
		motor4_off(&htim3);
		motor5_on(&htim3);
		motor6_off(&htim3);
	}else if(c == 'j'){
		motor1_on(&htim2);
		motor2_on(&htim2);
		motor3_off(&htim2);
		motor4_off(&htim3);
		motor5_on(&htim3);
		motor6_off(&htim3);
	}else if(c == 'k'){
		motor1_off(&htim2);
		motor2_off(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_off(&htim3);
		motor6_off(&htim3);
	}else if(c == 'l'){
		motor1_off(&htim2);
		motor2_off(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_on(&htim3);
		motor6_off(&htim3);
	}else if(c == 'm'){
		motor1_on(&htim2);
		motor2_off(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_off(&htim3);
		motor6_off(&htim3);
	}else if(c == 'n'){
		motor1_on(&htim2);
		motor2_on(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_off(&htim3);
		motor6_off(&htim3);
	}else if(c == 'o'){
		motor1_off(&htim2);
		motor2_on(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_off(&htim3);
		motor6_off(&htim3);
	}else if(c == 'p'){
		motor1_on(&htim2);
		motor2_off(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_on(&htim3);
		motor6_off(&htim3);
	}else if(c == 'q'){
		motor1_on(&htim2);
		motor2_on(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_on(&htim3);
		motor6_off(&htim3);
	}else if(c == 'r'){
		motor1_off(&htim2);
		motor2_on(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_on(&htim3);
		motor6_off(&htim3);
	}else if(c == 's'){
		motor1_on(&htim2);
		motor2_off(&htim2);
		motor3_on(&htim2);
		motor4_off(&htim3);
		motor5_on(&htim3);
		motor6_off(&htim3);
	}else if(c == 't'){
		motor1_on(&htim2);
		motor2_on(&htim2);
		motor3_on(&htim2);
		motor4_off(&htim3);
		motor5_on(&htim3);
		motor6_off(&htim3);
	}else if(c == 'u'){
		motor1_off(&htim2);
		motor2_off(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_off(&htim3);
		motor6_on(&htim3);
	}else if(c == 'v'){
		motor1_off(&htim2);
		motor2_off(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_on(&htim3);
		motor6_on(&htim3);
	}else if(c == 'w'){
		motor1_on(&htim2);
		motor2_on(&htim2);
		motor3_off(&htim2);
		motor4_off(&htim3);
		motor5_on(&htim3);
		motor6_on(&htim3);
	}else if(c == 'x'){
		motor1_on(&htim2);
		motor2_off(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_off(&htim3);
		motor6_on(&htim3);
	}else if(c == 'y'){
		motor1_on(&htim2);
		motor2_on(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_off(&htim3);
		motor6_on(&htim3);
	}else if(c == 'z'){
		motor1_off(&htim2);
		motor2_on(&htim2);
		motor3_on(&htim2);
		motor4_on(&htim3);
		motor5_off(&htim3);
		motor6_on(&htim3);
	}

}
char read(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==GPIO_PIN_SET) {
		return '1';
	} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==GPIO_PIN_SET) {
		return '4';
	} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==GPIO_PIN_SET) {
		return '7';
	} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==GPIO_PIN_SET) {
		return '*';
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==GPIO_PIN_SET) {
		return '2';
	} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==GPIO_PIN_SET) {
		return '5';
	} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==GPIO_PIN_SET) {
		return '8';
	} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==GPIO_PIN_SET) {
		return '0';
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==GPIO_PIN_SET) {
		return '3';
	} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==GPIO_PIN_SET) {
		return '6';
	} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==GPIO_PIN_SET) {
		return '9';
	} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==GPIO_PIN_SET) {
		return '#';
	}

	return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char initial_char = 0;
	int count = 0;
	char string[128] = {0};
	int top = 0;
	uint16_t value = 0;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
//  HAL_ADC_Start(&hadc1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	  // servo motor 1&2
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

	  if ((initial_char=read()) && ('1'<=initial_char) && (initial_char<='9')) {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		  HAL_Delay(100);
		  while (1) {
			  if (read()=='#' || count>=2) {
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
				  string[top++] = (initial_char-49)*3+ 97 + count;
				  HAL_Delay(300);
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				  HAL_Delay(100);
				  break;
			  } else if (read()==initial_char) {
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
				  HAL_Delay(100);
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				  HAL_Delay(100);
				  count++;
			  }
		  }
		  count = 0;
	  } else if (initial_char=='*' || top==128) {
		  for (int i=0; i<top; ++i) {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			  display(string[i]);

			  HAL_ADC_Start(&hadc1);
			  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			  value = HAL_ADC_GetValue(&hadc1)/4 + 30;
			  HAL_Delay(value);

			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			  HAL_Delay(70);
		  }
		  top = 0;
	  }

//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
//	  HAL_ADC_Start(&hadc1);
//	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	  value = HAL_ADC_GetValue(&hadc1)/10;
//	  HAL_Delay(value);
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
//	  HAL_Delay(100);

	}

}
/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
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

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 14400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 14400;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
