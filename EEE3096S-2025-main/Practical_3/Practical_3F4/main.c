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
#include "stm32f4xx.h"
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
//TODO: Define and initialise the global varibales required
/*
  start_time
  end_time
  execution_time
  checksum: should be uint64_t
  initial width and height maybe or you might opt for an array??
*/
/* Timing and result measurement variables */
uint32_t start_time = 0;       // Stores start time stamp (s)
uint32_t end_time = 0;         // Stores end time stamp (s)
uint32_t execution_time = 0;   // Calculated duration (s)
uint64_t checksum = 0;         // Accumulates iteration counts
uint32_t start_cycles = 0;    //cycle counter start
uint32_t end_cycles = 0;      //cycle counter end
uint32_t cycles_taken = 0;    // Total CPU cycles used
float throughput = 0.0f;     //pixels per second throughput

/* Benchmarking image dimensions */
const uint32_t IMAGE_DIMENSIONS[] = {128, 160, 192, 224, 256};
const uint32_t NUM_DIMENSIONS = 5;

/* Current test dimensions (initialized to smallest size) */
uint32_t width = IMAGE_DIMENSIONS[4];
uint32_t height = IMAGE_DIMENSIONS[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations);
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
  // Enable DWT cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;           // Enable trace
  DWT->CYCCNT = 0;                                          // Reset counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                      // Enable cycle counting

  //TODO: Turn on LED 0 to signify the start of the operation
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  // === Task 4 scalability loop ===
  //const uint32_t TEST_WIDTHS[]  = {360, 640, 800, 1024, 1280, 1920};
  //const uint32_t TEST_HEIGHTS[] = {192, 360, 450,  576,  720, 1080};
 // const uint32_t NUM_TESTS = 6;

  //for (int i = 0; i < NUM_TESTS; i++) {
	 // uint32_t width  = TEST_WIDTHS[i];
	  //uint32_t height = TEST_HEIGHTS[i];

      // Record start time + cycles
      start_time = HAL_GetTick();                          // Wall clock time
      start_cycles = DWT->CYCCNT;                          // CPU Cycle Counter

      //TODO: Call the Mandelbrot Function and store the output in the checksum variable defined initially
      checksum = calculate_mandelbrot_double(width, height, MAX_ITER);

      // Record end time + cycles
      end_time = HAL_GetTick();                             // Wall clock time
      end_cycles = DWT->CYCCNT;                             // CPU Cycle Counter

      // Calculate execution time + cycles
      execution_time = end_time - start_time;        // ms
      cycles_taken = end_cycles - start_cycles;      // cycles

      // Calculate throughput in pixels/sec
      uint32_t num_pixels = width * height;
      throughput = (float)num_pixels / ((float)execution_time / 1000.0f);     // px/s

      // Small pause between tests so Live Expressions can update
      HAL_Delay(1000);
  //}

  //TODO: Turn on LED 1 to signify the end of the operation
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  //TODO: Hold the LEDs on for a 2s delay
  HAL_Delay(2000);

  //TODO: Turn off the LEDs
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Stay idle — values visible in Live Expressions
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
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
// Mandelbrot with fixed point arithmetic
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
  uint64_t mandelbrot_sum = 0;
  const int32_t scale = 1000000;
  const int32_t escape_threshold = 4 * scale * 0.9;  // Tuned threshold

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int32_t x0 = ( (int64_t)x * 35 * scale / width ) / 10 - (25 * scale / 10);
      int32_t y0 = ( (int64_t)y * 20 * scale / height ) / 10 - (10 * scale / 10);

      int32_t xi = 0, yi = 0;
      int iteration = 0;

      while (iteration < max_iterations) {
        int64_t xi_sq = ((int64_t)xi * xi) / scale;
        int64_t yi_sq = ((int64_t)yi * yi) / scale;

        if (xi_sq + yi_sq > escape_threshold) break;

        int32_t temp = xi_sq - yi_sq + x0;
        yi = ( (int64_t)2 * xi * yi ) / scale + y0;
        xi = temp;
        iteration++;
      }
      mandelbrot_sum += iteration;
    }
  }
  return mandelbrot_sum;
}

// Mandelbrot with double
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations){
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
// Mandelbrot with float
uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations) {
    uint64_t mandelbrot_sum = 0;
    const float escape_threshold = 4.0f * 0.9f;   // same scaled threshold

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Map pixel (x, y) to complex plane [-2.5, 1.0] × [-1.0, 1.0]
            float x0 = (x * 3.5f / (float)width) - 2.5f;
            float y0 = (y * 2.0f / (float)height) - 1.0f;

            float xi = 0.0f, yi = 0.0f;
            int iteration = 0;

            while (iteration < max_iterations) {
                float xi_sq = xi * xi;
                float yi_sq = yi * yi;

                if (xi_sq + yi_sq > escape_threshold) break;

                float temp = xi_sq - yi_sq + x0;
                yi = 2.0f * xi * yi + y0;
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
  while (1)
  {
  }
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
