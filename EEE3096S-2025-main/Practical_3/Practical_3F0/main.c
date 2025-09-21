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
#include <stdint.h>
#include "stm32f0xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MAX_ITER 100

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t start_time = 0;
uint32_t end_time = 0;
uint32_t execution_time = 0;
uint64_t checksum = 0;

const uint32_t IMAGE_DIMENSIONS[] = {128, 160, 192, 224, 256};
const uint32_t NUM_DIMENSIONS = 5;
uint32_t width = IMAGE_DIMENSIONS[0];
uint32_t height = IMAGE_DIMENSIONS[0];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  // Visual indicator: LED0 ON (start)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  // Record start time
  start_time = HAL_GetTick();

  // === Call Mandelbrot function (choose one) ===
  //checksum = calculate_mandelbrot_fixed_point_arithmetic(width, height, MAX_ITER);
  // OR
  checksum = calculate_mandelbrot_double(width, height, MAX_ITER);

  // Record end time
  end_time = HAL_GetTick();
  execution_time = end_time - start_time;

  // Visual indicator: LED1 ON (end)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  // Hold LEDs ON for 2 seconds
  HAL_Delay(2000);

  // Turn off LEDs
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  while (1)
  {
    // Idle loop
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
// Mandelbrot functions (untouched, just brought in)
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations) {
  uint64_t mandelbrot_sum = 0;
  const int32_t scale = 1000000;
  const int32_t escape_threshold = 4 * scale * 0.9;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int32_t x0 = ((int64_t)x * 35 * scale / width) / 10 - (25 * scale / 10);
      int32_t y0 = ((int64_t)y * 20 * scale / height) / 10 - (10 * scale / 10);

      int32_t xi = 0, yi = 0;
      int iteration = 0;

  while (iteration < max_iterations) {
        int64_t xi_sq = ((int64_t)xi * xi) / scale;
        int64_t yi_sq = ((int64_t)yi * yi) / scale;

        if (xi_sq + yi_sq > escape_threshold) break;

        int32_t temp = xi_sq - yi_sq + x0;
        yi = ((int64_t)2 * xi * yi) / scale + y0;
        xi = temp;
        iteration++;
      }
      mandelbrot_sum += iteration;
    }
  }
  return mandelbrot_sum;
}

uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations) {
  uint64_t mandelbrot_sum = 0;
  const double escape_threshold = 4.0 * 0.9;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      double x0 = (x * 3.5 / width) - 2.5;
      double y0 = (y * 2.0 / height) - 1.0;

      double xi = 0.0, yi = 0.0;
      int iteration = 0;

      while (iteration < max_iterations) {
        double xi_sq = xi * xi;
        double yi_sq = yi * yi;

        if (xi_sq + yi_sq > escape_threshold) break;

        double temp = xi_sq - yi_sq + x0;
        yi = 2.0 * xi * yi + y0;
        xi = temp;
        iteration++;
      }
      mandelbrot_sum += iteration;
    }
  }
  return mandelbrot_sum;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
