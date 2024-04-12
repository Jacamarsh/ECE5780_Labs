/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 1
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	
	
	//Lab 1, part 1 check-off main method: (comment/uncomment it out when doing eahc checkoff) --> Red & Blue toggle on/off given delay
	/*
	
int main(void) {
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	// Enable GPIOC peripheral clock using AHB peripheral clock enable register
	// Enable the GPIOC clock in the RCC via: Enable AHB peripheral clock register with GPIOC enable register
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	

	//Configure LED pins (PC6-PC9) in the following way:
	//General-purpose output mode using the MODER register.
	GPIOC->MODER |= (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18);
	GPIOC->MODER &= ~((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));
	
	//Push-pull output type using the OTYPER register.
	// (Is in this mode by default, no change needed)
	
	//Low speed using the OSPEEDR register.
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18));
	
	//No pull-up/down resistors using the PUPDR register.
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15) | (1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));

	//USER Button pin (PA0) should be configured to:
	//Digital input mode using the MODER register.
	// (Is in this mode by default, no change needed)
	
	//Low speed using the OSPEEDR register.
	GPIOA->OSPEEDR &= ~(1 << 0);
	
	//Pull-down resistor using the PUPDR register.
	GPIOA->PUPDR |= (1 << 1);
	GPIOA->PUPDR &= ~(1 << 0);
													
															
	//PC6 (red) to high, PC7 (blue) to low
	GPIOC->BSRR |= (1 << 6); // initial: PC6 (red) to high: set to 1
	GPIOC->BSRR &= ~(1 << 7); // initial: PC7 (blue) to low: clear to 0
	
	while (1) {
		HAL_Delay(200); // Delay 200ms
		
		// Toggle the output state of both PC6 and PC7	
		GPIOC->ODR ^= (1 << 7); // Toggle state (LED being on/off) of PC8 (blue)
		GPIOC->ODR ^= (1 << 6); // Toggle state (LED being on/off) of PC9 (red)		
	}
	
}

*/


	
	
	
	
	
	//Lab 1, part 2 check-off main method: (comment/uncomment it out when doing eahc checkoff) --> Red/blue LEDs toggle given USER btn hit 
	
	int main(void) {
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	// Enable GPIOC peripheral clock using AHB peripheral clock enable register
	// Enable the GPIOC clock in the RCC via: Enable AHB peripheral clock register with GPIOC enable register
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	// Enable GPOIOA clock in the RCC for button
	 RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	//Configure LED pins (PC6-PC9) in the following way:
	//General-purpose output mode using the MODER register.
	GPIOC->MODER |= (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18);
	GPIOC->MODER &= ~((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));
	
	//Push-pull output type using the OTYPER register.
	GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
	// (Is in this mode by default, no change needed)
	
	//Low speed using the OSPEEDR register.
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18));
	
	//No pull-up/down resistors using the PUPDR register.
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15) | (1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));

	//USER Button pin (PA0) should be configured to:
	//Digital input mode using the MODER register.
	 GPIOA->MODER &= ~((1<<0) | (1<<1));
	
	//Low speed using the OSPEEDR register.
	GPIOA->OSPEEDR &= ~(1 << 0);
	
	//Pull-down resistor using the PUPDR register.
	GPIOA->PUPDR |= (1 << 0);
	GPIOA->PUPDR &= ~(1 << 1);
													
															
	//PC6 (red) to high, PC7 (blue) to low
	GPIOC->BSRR |= (1 << 6); // initial: PC6 (red) to high: set to 1
	GPIOC->BSRR &= ~(1 << 7); // initial: PC7 (blue) to low: clear to 0
	
	uint8_t button_down = 0;
	uint32_t debouncer = 0;
while(1) {
	HAL_Delay(5); // Delay 20ms
	debouncer = (debouncer << 1); // Always shift every loop iteration
	if ((GPIOA->IDR & (1 << 0)) == 0) { // If input signal is set/high
		debouncer |= 0x01; // Set lowest bit of bit-vector
	}
	if (debouncer == 0xFFFFFFFF) {
	if(!button_down) {
				GPIOC->ODR ^= (1 << 7); // Toggle state (LED being on/off) of PC8 (blue)
				GPIOC->ODR ^= (1 << 6); // Toggle state (LED being on/off) of PC9 (red)
				button_down = 1;
			}			
		}
		else {
			button_down = 0;
		}
	
	if (debouncer == 0x00000000) {
	// This code triggers repeatedly when button is steady low!
	//GPIOC->ODR ^= (1 << 7); // Toggle state (LED being on/off) of PC8 (blue)
	}
	if (debouncer == 0x7FFFFFFF) {
	// This code triggers only once when transitioning to steady high!
	//GPIOC->ODR ^= (1 << 7); // Toggle state (LED being on/off) of PC8 (blue)
	}
	// When button is bouncing the bit-vector value is random since bits are set when the button is high and not when it bounces low.
	}
	
	
	//uint32_t idr_value = ~GPIOB->IDR;
	
}
	
	


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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

