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
#include <stdio.h>

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

//Sine Wave
const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
//Square Wave
const uint8_t square_table[32] = {254,254,254,254,254,254,254,254,254,254,
254,254,254,254,254,254,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//Sawtooth Wave
const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,
111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};
//Triangle Wave
const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};


	
uint32_t index = 0;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init(); // Initialize Hardware Abstraction Layer

  SystemClock_Config(); // Configure the system clock

  // Enable GPIO and DAC clocks
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  // Initialize LEDs
  GPIO_InitTypeDef iniStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
                             GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &iniStr);
  
  // Initialize ADC input pin
  GPIO_InitTypeDef adc = {GPIO_PIN_4, GPIO_MODE_ANALOG, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &adc);
  
  // Initialize DAC output pin
  GPIO_InitTypeDef dac = {GPIO_PIN_4, GPIO_MODE_ANALOG, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
  HAL_GPIO_Init(GPIOA, &dac);
  
  // Enable ADC clock
  __HAL_RCC_ADC1_CLK_ENABLE();
  // Set ADC configuration
  ADC1->CFGR1 |= (1 << 4); // 8-bit resolution
  ADC1->CFGR1 |= (1 << 13); // Enable continuous conversion mode
  ADC1->CHSELR |= (1 << 14); // Select channel 14 (PC4)
  
  ADC1->CR |= (1 << 31); // Start ADC calibration
  while(1)
  {
    if(~(ADC1->CR) & (1U << 31)) { break; } // Wait for calibration to finish
  }
  ADC1->CR |= (0x1); // Enable ADC
  while(1) {
    if(ADC1->ISR & (0x1)) { break; } // Wait for ADC ready flag
  }
  ADC1->CR |= (1 << 2); // Start ADC conversion
  
  // Enable DAC clock
  __HAL_RCC_DAC1_CLK_ENABLE();
  DAC1->CR |= (1 << 0); // Enable DAC
  
  while (1)
  {
    // Read ADC value
    uint8_t ADCData = (ADC1->DR & 0xFF);
    
    // Control LEDs based on ADC value
    if(ADCData > 50) {
      GPIOC->ODR |= (1 << 7);
    } else {
      GPIOC->ODR &= ~(1 << 7);
    }
    if(ADCData > 100) {
      GPIOC->ODR |= (1 << 8);
    } else {
      GPIOC->ODR &= ~(1 << 8);
    }
    if(ADCData > 150) {
      GPIOC->ODR |= (1 << 6);
    } else {
      GPIOC->ODR &= ~(1 << 6);
    }
    if(ADCData > 200) {
      GPIOC->ODR |= (1 << 9);
    } else {
      GPIOC->ODR &= ~(1 << 9);
    }
    
    // Output waveform to DAC
    DAC1->DHR8R1 = sine_table[index];
    index++;
    if(index > 31) {
      index = 0;
    }
    
    HAL_Delay(1); // Delay
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

  // Initialize RCC Oscillators
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // Initialize CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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
  // User can add his own implementation to report the HAL error return state
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
