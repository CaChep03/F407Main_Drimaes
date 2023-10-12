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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "pid.h"
#include "math.h"
// #include "tca9548a.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID_TypeDef RPID;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t PWM_Freq = 5000;

// variables for MPU data
uint8_t Buffer[11];
bool check = 1;
float theta = 0;

// variables for UART to Swerve module
uint16_t temp_angle = 300;

// variables for I2C
uint16_t SWERVE_DRIVE_1_ADDR = 0x15 << 1;
uint8_t I2C_Tx_Buffer[] = {0x05, 0x00, 0x00};
uint8_t Scan_arr[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Buzzer_Beep(uint8_t times, uint8_t duration)
{
  for (uint8_t i = 0; i < times; i++)
  {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_Delay(duration);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(duration);
  }
}

void MPU_Data_Merge(bool Flag)
{
  if (Flag)
  {
    if (Buffer[0] == 0x55 && Buffer[1] == 0x53)
    {
      Flag = false;
      theta = (Buffer[7] << 8 | Buffer[6]) / 32768.0 * 180;
    }
    else
    {
      return;
    }
  }
}

// 0 to disable PWM
void PWM_Gen(uint8_t channel, uint32_t duty)
{
  uint32_t period = (SystemCoreClock / PWM_Freq) - 1;
  htim8.Instance->ARR = period;
  switch (channel)
  {
  case 1:
    duty == 0 ? HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1) : HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    htim8.Instance->CCR1 = (period * duty) / 100;
    break;
  case 2:
    duty == 0 ? HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2) : HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    htim8.Instance->CCR2 = (period * duty) / 100;
    break;
  case 3:
    duty == 0 ? HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3) : HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    htim8.Instance->CCR3 = (period * duty) / 100;
    break;
  case 4:
    duty == 0 ? HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4) : HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    htim8.Instance->CCR4 = (period * duty) / 100;
    break;
  case 0:
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Stop_IT(&htim8);
    break;
  default:
    break;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
  }
}

void Div16_To_8(uint16_t data, uint8_t *data_array)
{
  *(data_array + 1) = (uint8_t)((data & 0xFF00) >> 8);
  *(data_array + 2) = (uint8_t)(data & 0x00FF);
}

void I2C_Scan()
{
  uint8_t i;
  HAL_StatusTypeDef res;

  for (i = 0; i < 128; i++)
  {
    res = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 1, 100);
    if (res == HAL_OK)
    {
      Scan_arr[i] = 1;
    }
    else
    {
      Scan_arr[i] = 0;
    }
  }
  if (Scan_arr[21] == 0)
  {
    while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(21 << 1), 1, 100) != HAL_OK)
    {
      Buzzer_Beep(1, 50);
    }
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
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  I2C_Scan();
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(10);

    Div16_To_8(100, I2C_Tx_Buffer);
    HAL_I2C_Master_Transmit(&hi2c1, SWERVE_DRIVE_1_ADDR, I2C_Tx_Buffer, 3, 10);
    HAL_Delay(5000);
    Div16_To_8(200, I2C_Tx_Buffer);
    HAL_I2C_Master_Transmit(&hi2c1, SWERVE_DRIVE_1_ADDR, I2C_Tx_Buffer, 3, 10);
    HAL_Delay(5000);
    Div16_To_8(90, I2C_Tx_Buffer);
    HAL_I2C_Master_Transmit(&hi2c1, SWERVE_DRIVE_1_ADDR, I2C_Tx_Buffer, 3, 10);
    HAL_Delay(5000);
    Div16_To_8(270, I2C_Tx_Buffer);
    HAL_I2C_Master_Transmit(&hi2c1, SWERVE_DRIVE_1_ADDR, I2C_Tx_Buffer, 3, 10);
    HAL_Delay(5000);
    Buzzer_Beep(1, 5);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

#ifdef USE_FULL_ASSERT
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
