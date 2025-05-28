/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  STATE_IDLE,
  STATE_STARTUP,
  STATE_GENERATE_AND_DISPLAY_SEQUENCE,
  STATE_WAIT_FOR_PLAYER_INPUT,
  STATE_EVALUATE_SEQUENCE,
  STATE_SUCCESS_BLINK,
  STATE_FAILURE_BLINK
} GameState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEQUENCE_LENGTH 4
#define DEBOUNCE_TIME_MS 200
#define LED_FEEDBACK_DURATION_MS 300
#define BUZZER_DURATION 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Global variables to manage the game state and logic
volatile GameState_t g_gameState = STATE_IDLE;
volatile bool g_gameRunning = false;

uint8_t g_generatedSequence[SEQUENCE_LENGTH];
uint8_t g_playerSequence[SEQUENCE_LENGTH];
volatile uint8_t g_currentPlayerInputIndex = 0; 

volatile uint8_t g_ledFeedbackActive = 0; // 0 = no feedback, 1 = red LED, 2 = green LED, 3 = yellow LED
volatile uint32_t g_ledFeedbackStartTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void PrepareForNextInput(void);
static void EnableGameButtonInterrupts(void);
static void DisableGameButtonInterrupts(void);
static void EnableStartButtonInterrupt(void);
static void DisableStartButtonInterrupt(void);
static void BuzzerBeep(uint32_t duration);
static void DisplaySequence(uint8_t ledValue, uint32_t onDuration, uint32_t offDuration);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//EXTI0_1_IRQn handler for RED and GREEN buttons
//EXTI2_3_IRQn handler for YELLOW button
//EXTI4_15_IRQn handler for START button

static void EnableGameButtonInterrupts(void)
{
    __HAL_GPIO_EXTI_CLEAR_IT(RED_BUTTON_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(GREEN_BUTTON_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(YELLOW_BUTTON_Pin);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}

static void DisableGameButtonInterrupts(void)
{
    HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
    HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
}

static void EnableStartButtonInterrupt(void)
{
    __HAL_GPIO_EXTI_CLEAR_IT(START_BUTTON_Pin);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

static void DisableStartButtonInterrupt(void)
{
    HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
}

static void PrepareForNextInput(void)
{
    if (g_currentPlayerInputIndex < SEQUENCE_LENGTH)
    {
        EnableGameButtonInterrupts();
    }
}

//Buzzer function to beep for a specified duration
static void BuzzerBeep(uint32_t duration)
{
	for (int i = 0; i < 5; i++)
	{
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		HAL_Delay(duration);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		HAL_Delay(duration);
	}
}

static void DisplaySequence(uint8_t ledValue, uint32_t onDuration, uint32_t offDuration)
{
  switch (ledValue)
  {
    case 0: //red led
      HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
      HAL_Delay(onDuration);
      HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
      HAL_Delay(offDuration);
      break;
    case 1: //green led
      HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
      HAL_Delay(onDuration);
      HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
      HAL_Delay(offDuration);
      break;
    case 2: //yellow led
      HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_SET);
      HAL_Delay(onDuration);
      HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);
      HAL_Delay(offDuration);
      break;
  }
}
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  srand(HAL_GetTick());

  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    switch (g_gameState)
    {
      case STATE_IDLE:

        DisableGameButtonInterrupts();
        EnableStartButtonInterrupt();
        g_gameRunning = false;
        g_currentPlayerInputIndex = 0;
        for (int i = 0; i < SEQUENCE_LENGTH; i++)
        {
          g_playerSequence[i] = 0;
        }
        HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);
        break;

      case STATE_STARTUP:

        DisableGameButtonInterrupts();
        for (int i = 0; i < 3; i++)
        {
          HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
          HAL_Delay(500);
          HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
          HAL_Delay(500);
          HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_SET);
          HAL_Delay(500);
          HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);
        }
        HAL_Delay(1000);

        g_gameState = STATE_GENERATE_AND_DISPLAY_SEQUENCE;
        break;

      case STATE_GENERATE_AND_DISPLAY_SEQUENCE:

        DisableGameButtonInterrupts();
        g_currentPlayerInputIndex = 0;

        for (int i = 0; i < SEQUENCE_LENGTH; i++)
        {
          g_generatedSequence[i] = rand() % 3;
        }

        for (int i = 0; i < SEQUENCE_LENGTH; i++)
        {
          DisplaySequence(g_generatedSequence[i], 500, 500); // Display each LED in the sequence
        }
        HAL_Delay(2000);
        g_gameState = STATE_WAIT_FOR_PLAYER_INPUT;
        break;

      case STATE_WAIT_FOR_PLAYER_INPUT:

        PrepareForNextInput();

        if (g_ledFeedbackActive > 0 && g_currentPlayerInputIndex < SEQUENCE_LENGTH)
        {
          uint32_t currentTime = HAL_GetTick();
          if (currentTime - g_ledFeedbackStartTime >= LED_FEEDBACK_DURATION_MS)
          {

            HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);
            g_ledFeedbackActive = 0;
          }
        }
        break;

      case STATE_EVALUATE_SEQUENCE:

        DisableGameButtonInterrupts();

        if (g_ledFeedbackActive > 0)
        {
            uint32_t currentTimeEval = HAL_GetTick();
            if (currentTimeEval - g_ledFeedbackStartTime < LED_FEEDBACK_DURATION_MS)
            {
                HAL_Delay(LED_FEEDBACK_DURATION_MS - (currentTimeEval - g_ledFeedbackStartTime));
            }
        }

        HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);
        g_ledFeedbackActive = 0;

        _Bool sequenceMatch = 1;
        for (int i = 0; i < SEQUENCE_LENGTH; i++)
        {
          if (g_generatedSequence[i] != g_playerSequence[i])
          {
            sequenceMatch = 0;
            break;
          }
        }

        HAL_Delay(1000);
        if (sequenceMatch) { g_gameState = STATE_SUCCESS_BLINK; }
        else { g_gameState = STATE_FAILURE_BLINK; }
        break;

      case STATE_SUCCESS_BLINK:

    	EnableStartButtonInterrupt();
        for (int i = 0; i < 2; i++)
        {
          HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_SET);
          HAL_Delay(250);
          HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_RESET);
          HAL_Delay(250);
        }
        HAL_Delay(1000);
        g_gameState = STATE_GENERATE_AND_DISPLAY_SEQUENCE;
        break;

      case STATE_FAILURE_BLINK:

    	  BuzzerBeep(BUZZER_DURATION);
        for (int i = 0; i < 3; i++)
        {
          HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_SET);
          HAL_Delay(250);
          HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_RESET);
          HAL_Delay(250);
        }
        HAL_Delay(1000);
        g_gameState = STATE_GENERATE_AND_DISPLAY_SEQUENCE;
        break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|INDICATOR_LED_Pin|GREEN_LED_Pin|RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, YELLOW_LED_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RED_BUTTON_Pin GREEN_BUTTON_Pin */
  GPIO_InitStruct.Pin = RED_BUTTON_Pin|GREEN_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin INDICATOR_LED_Pin GREEN_LED_Pin RED_LED_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|INDICATOR_LED_Pin|GREEN_LED_Pin|RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : START_BUTTON_Pin */
  GPIO_InitStruct.Pin = START_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(START_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : YELLOW_LED_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = YELLOW_LED_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : YELLOW_BUTTON_Pin */
  GPIO_InitStruct.Pin = YELLOW_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(YELLOW_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t lastButtonPressTime = 0;
  static uint32_t startButtonPressTime = 0;
  uint32_t currentTime = HAL_GetTick();

  if (currentTime - lastButtonPressTime < DEBOUNCE_TIME_MS) 
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    return;
  }

  if (currentTime - startButtonPressTime < DEBOUNCE_TIME_MS)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    return;
  }

  if (g_gameState == STATE_WAIT_FOR_PLAYER_INPUT && g_currentPlayerInputIndex < SEQUENCE_LENGTH)
  {
    DisableGameButtonInterrupts();

    lastButtonPressTime = currentTime;

    if (GPIO_Pin == RED_BUTTON_Pin)
    {
      g_playerSequence[g_currentPlayerInputIndex] = 0;
      HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
      g_ledFeedbackActive = 1;
      g_ledFeedbackStartTime = currentTime;
    }
    else if (GPIO_Pin == GREEN_BUTTON_Pin)
    {
      g_playerSequence[g_currentPlayerInputIndex] = 1;
      HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
      g_ledFeedbackActive = 2;
      g_ledFeedbackStartTime = currentTime;
    }
    else if (GPIO_Pin == YELLOW_BUTTON_Pin)
    {
      g_playerSequence[g_currentPlayerInputIndex] = 2;
      HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_SET);
      g_ledFeedbackActive = 3;
      g_ledFeedbackStartTime = currentTime;
    }
    else
    {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
      return;
    }

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);

    g_currentPlayerInputIndex++;

    if ( g_currentPlayerInputIndex >= SEQUENCE_LENGTH )
    {
      g_gameState = STATE_EVALUATE_SEQUENCE;
    }
  }
  else if ( g_gameState == STATE_IDLE )
  {
    DisableStartButtonInterrupt();

    if (GPIO_Pin == RED_BUTTON_Pin || GPIO_Pin == GREEN_BUTTON_Pin || GPIO_Pin == YELLOW_BUTTON_Pin)
    {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
      return;
    }
    else if (GPIO_Pin == START_BUTTON_Pin)
    {
      startButtonPressTime = currentTime;
      if (!g_gameRunning)
      {
        g_gameRunning = true;
        g_gameState = STATE_STARTUP;
        g_currentPlayerInputIndex = 0;
        for (int i = 0; i < SEQUENCE_LENGTH; i++)
        {
          g_playerSequence[i] = 0;
        }
      }
      else
      {
        g_gameRunning = false;
        g_gameState = STATE_IDLE;
      }
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
  }
  else
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
  }
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
