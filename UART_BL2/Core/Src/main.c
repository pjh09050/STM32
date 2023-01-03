/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TxBufferSize	(countof(TxBuffer) - 1)
#define RxBufferSize	5
#define countof(a)   (sizeof(a) / sizeof(*(a)))
#define sw01_size   (countof(sw01txbuffer) -1)
#define sw02_size   (countof(sw02txbuffer) -1)
#define sw03_size   (countof(sw03txbuffer) -1)
#define sw04_size   (countof(sw04txbuffer) -1)

uint8_t sw01txbuffer[] = "<S11>";
uint8_t sw02txbuffer[] = "<S22>";
uint8_t sw03txbuffer[] = "<S33>";
uint8_t sw04txbuffer[] = "<S44>";
uint8_t base_protocol =0x00;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

uint8_t TxBuffer[] = "\n\rUART-BlueTooth Example\n\r\n\r";
uint8_t RxBuffer[RxBufferSize];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

void LED_Pin_Mode(uint16_t gpio_pin, unsigned char on_off);
void LED_OnOff(int, int);

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
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  LED_OnOff(LED_ALL, 500);
  HAL_UART_Transmit(&huart4, (uint8_t*)TxBuffer, TxBufferSize, 0xFFFF);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// -- PC에서 보낸 메시지를 받는다.
		HAL_UART_Receive_IT(&huart4, (uint8_t*)RxBuffer,sizeof(RxBuffer));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PG0 PG1 PG2 PG3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

void LED_OnOff(int led, int interval)
{
	HAL_GPIO_WritePin(GPIO_LED, led, GPIO_PIN_SET );
	HAL_GPIO_WritePin(GPIO_LED_Nucleo, led, GPIO_PIN_SET );

	HAL_Delay(interval);

	HAL_GPIO_WritePin(GPIO_LED, led, GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIO_LED_Nucleo, led, GPIO_PIN_RESET );
}

// -- 수신 데이터가 들어오면 호출되는 callback 함수
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart4)
{

	// -- 꺽쇠'<>'안의 첫번째 비트를 검사
	if(RxBuffer[1] == 'L') {
		// -- 꺽쇠'<>'안의 두번째 비트를 검사
		switch(RxBuffer[2])	{
			case '1' : LED_Pin_Mode(GPIO_PIN_0, RxBuffer[3]);	break;
			case '2' : LED_Pin_Mode(GPIO_PIN_1, RxBuffer[3]);	break;
			case '3' : LED_Pin_Mode(GPIO_PIN_2, RxBuffer[3]);	break;
			case '4' : LED_Pin_Mode(GPIO_PIN_3, RxBuffer[3]);	break;
			case '5' : LED_Pin_Mode(GPIO_PIN_4, RxBuffer[3]);	break;
			case '6' : LED_Pin_Mode(GPIO_PIN_5, RxBuffer[3]);	break;
			case '7' : LED_Pin_Mode(GPIO_PIN_6, RxBuffer[3]);	break;
			case '8' : LED_Pin_Mode(GPIO_PIN_7, RxBuffer[3]);	break;

			default:   break;
		}
	}
}

	// -- 꺽쇠'<>'안의 3번째 비트에 따라 LED를 ON/OFF 한다.
void LED_Pin_Mode(uint16_t gpio_pin, unsigned char on_off)
{
	switch(on_off) {
		case '0': HAL_GPIO_WritePin(GPIO_LED,gpio_pin,GPIO_PIN_RESET);  break;
		case '1': HAL_GPIO_WritePin(GPIO_LED,gpio_pin,GPIO_PIN_SET);    break;
		default: break;
	}
}


// -- SW가 눌러지면 해당되는 메시지를 PC로 전송한다.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{

if ( GPIO_PIN == SW1)
	HAL_UART_Transmit(&huart4, (uint8_t*)sw01txbuffer, sw01_size , 0xFFFF);
else if ( GPIO_PIN == SW2)
	HAL_UART_Transmit(&huart4, (uint8_t*)sw02txbuffer, sw02_size , 0xFFFF);
else if ( GPIO_PIN == SW3)
	HAL_UART_Transmit(&huart4, (uint8_t*)sw03txbuffer, sw03_size , 0xFFFF);
else if ( GPIO_PIN == SW4)
	HAL_UART_Transmit(&huart4, (uint8_t*)sw04txbuffer, sw04_size , 0xFFFF);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
