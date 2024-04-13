/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 2
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


// Declaring EXTI0_1_IRQ's Handler Function
void EXTIO_1_IRQHandler (void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---- -----------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


//-----------------------------------------------------------

/**
  * @brief  The application entry point.
  * @retval int
  */
	
	/*		First check-off methods
	
int main(void)
{
  // Reset of all peripherals, Initializes the Flash interface and the Systick. 
  HAL_Init();

  // Configure the system clock 
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	__HAL_RCC_GPIOA_CLK_ENABLE(); // Enable the GPIOA clock in the RCC
	
	// Setting PC6-9 (all 4 LEDs) to general purpose mode
	GPIOC->MODER |= (1<<6*2) | (1<<7*2) | (1<<8*2) | (1<<9*2);
	
  SystemClock_Config();
	
	GPIOC->ODR |= (1<<9); // setting PC9 (green LED) to high
	
	//setting PA0 (User btn) to input mode
	GPIOA->MODER &= ~((1<<0) | (1<<1)); 	
	
	// setting PAO (user btn) to low-speed
	GPIOA->OSPEEDR |= (1<<0); 
	
	// setting PA0 (User btn) to pull-down mode --> low, then then (to VCC) when pressed
	GPIOA->PUPDR &= ~(1<<0);
	GPIOA->PUPDR |= (1<<1);
	
	//4. Configure the multiplexer to route PA0 to the EXTI input line 0 (EXTI0).
	// EXTI line 0 enabled (handles external interrupts from GPIO pins)
	//interrupt request not masked
	EXTI->IMR |= (1<<0); 
	
	// EXTI line 0 configured to trigger on: rising edge
	// When PAO goes from low to high, triggers interrupt
	// Assuming since btn starts low: interrupt when btn is just being pressed down
	EXTI->RTSR |= (1<<0); 
	
	// For EXTI0, all PA0, PB0, PC0, etc to 16 are all grouped up using multiplexors to EXTI0
	// Yet, we need configuration with actual button (PA0) to get external interrupt working
	// So: configure SYSCFG pin multiplexers to connect 2 signals together:
	//1. Use the RCC to enable the peripheral clock to the SYSCFG peripheral.
			// APB2 Enable register for RCC set to its SYSCFG comparator and clock enable
			// aka, enables clock for SYSCFG peripheral, which used to configure EXTI lines
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

	//2. Determine which SYSCFG multiplexer can route PA0 to the EXTI peripheral.
	// The multiplexer that can route PA0 to the EXTI peripheral is: SYSCFG_EXTICR1

	//3. Each of the EXTICRx registers control multiple pin multiplexers. Find which register contains
	//the configuration bits for the required multiplexer.
	// PA0 connected to EXTI0, so configure multiplexer for EXTI0.
	// Configuratio bits for EXTI0 --> Located in EXTICR[0] register
	
	
	
	
	//4. Configure the multiplexer to route PA0 to the EXTI input line 0 (EXTI0).
	// Selects PA0 (user btn) as source input for EXTI line 0
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0_Msk); // Clear EXTI0 bits
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // Set PA0 to EXITI0
	
	 //Enables EXTI_1_IRQn interrupt in the NVIC. Interrupt number for lines 0 and 1: EXTI0_1_IRQn
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	
	// Sets priority if EXTI0_1_IRQn interrupt to 1 (the lower the number, the higher the priority)
	NVIC_SetPriority(EXTI0_1_IRQn, 1);
	// Sets priority of SysTick interrupt to 2 (interrupt used for system timing)
	//NVIC_SetPriority(SysTick_IRQn, 2);
	// Assumed: Default interrupt priority 1: Inhabited by the reset button (RESET btn comes first! duh!)
	
	
	// 2.5 - Writing the EXTI Interrupt Handler
	//1. Handler name we declared: EXTI0_1_IRQn
	//2. Declare handler function (we wrote the method below main in this main.c file, and declared it above main.c) 
	
	
	
  while (1)
  {
		HAL_Delay(400);
		// Toggles PC6 (Red LED)
		GPIOC->ODR ^= (1<<6);
  }

	
}

// While pre-generated interrupt handlers exist in stm32f0xx_it.h,
// They can be declared anywhere, so we will put them here!
void EXTI0_1_IRQHandler(void){
	GPIOC->ODR ^= (1<<8) | (1<<9); // Toggling PC8 and PC9 (green and orange LEDs)
	
	// Clearing flag for input line 0 of EXTI Pending Register
	// Done to prevent looping. basically acknowledges its done, no longer pending
	EXTI->PR |= (1<<0); 
}
//-----------------------------------------------------------

*/




















//-----------------------------------------------------------

/**
  * @brief  The application entry point.
  * @retval int
  */
	
	//				2nd Check-off methods
int main(void)
{
  // Reset of all peripherals, Initializes the Flash interface and the Systick. 
  HAL_Init();

  // Configure the system clock 
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	__HAL_RCC_GPIOA_CLK_ENABLE(); // Enable the GPIOA clock in the RCC
	
	// Setting PC6-9 (all 4 LEDs) to general purpose mode
	GPIOC->MODER |= (1<<6*2) | (1<<7*2) | (1<<8*2) | (1<<9*2);
	
  SystemClock_Config();
	
	GPIOC->ODR |= (1<<9); // setting PC9 (green LED) to high
	
	//setting PA0 (User btn) to input mode
	GPIOA->MODER &= ~((1<<0) | (1<<1)); 	
	
	// setting PAO (user btn) to low-speed
	GPIOA->OSPEEDR |= (1<<0); 
	
	// setting PA0 (User btn) to pull-down mode --> low, then then (to VCC) when pressed
	GPIOA->PUPDR &= ~(1<<0);
	GPIOA->PUPDR |= (1<<1);
	
	//4. Configure the multiplexer to route PA0 to the EXTI input line 0 (EXTI0).
	// EXTI line 0 enabled (handles external interrupts from GPIO pins)
	//interrupt request not masked
	EXTI->IMR |= (1<<0); 
	
	// EXTI line 0 configured to trigger on: rising edge
	// When PAO goes from low to high, triggers interrupt
	// Assuming since btn starts low: interrupt when btn is just being pressed down
	EXTI->RTSR |= (1<<0); 
	
	// For EXTI0, all PA0, PB0, PC0, etc to 16 are all grouped up using multiplexors to EXTI0
	// Yet, we need configuration with actual button (PA0) to get external interrupt working
	// So: configure SYSCFG pin multiplexers to connect 2 signals together:
	//1. Use the RCC to enable the peripheral clock to the SYSCFG peripheral.
			// APB2 Enable register for RCC set to its SYSCFG comparator and clock enable
			// aka, enables clock for SYSCFG peripheral, which used to configure EXTI lines
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

	//2. Determine which SYSCFG multiplexer can route PA0 to the EXTI peripheral.
	// The multiplexer that can route PA0 to the EXTI peripheral is: SYSCFG_EXTICR1

	//3. Each of the EXTICRx registers control multiple pin multiplexers. Find which register contains
	//the configuration bits for the required multiplexer.
	// PA0 connected to EXTI0, so configure multiplexer for EXTI0.
	// Configuratio bits for EXTI0 --> Located in EXTICR[0] register
	
	
	
	
	//4. Configure the multiplexer to route PA0 to the EXTI input line 0 (EXTI0).
	// Selects PA0 (user btn) as source input for EXTI line 0
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0_Msk); // Clear EXTI0 bits
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // Set PA0 to EXITI0
	
	 //Enables EXTI_1_IRQn interrupt in the NVIC. Interrupt number for lines 0 and 1: EXTI0_1_IRQn
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	
	// Sets priority if EXTI0_1_IRQn interrupt to 1 (the lower the number, the higher the priority)
	NVIC_SetPriority(EXTI0_1_IRQn, 3);
	// Sets priority of SysTick interrupt to 2 (interrupt used for system timing)
	NVIC_SetPriority(SysTick_IRQn, 2);
	// Assumed: Default interrupt priority 1: Inhabited by the reset button (RESET btn comes first! duh!)
	
	
	// 2.5 - Writing the EXTI Interrupt Handler
	//1. Handler name we declared: EXTI0_1_IRQn
	//2. Declare handler function (we wrote the method below main in this main.c file, and declared it above main.c) 
	
	
	
  while (1)
  {
		HAL_Delay(400);
		// Toggles PC6 (Red LED)
		GPIOC->ODR ^= (1<<6);
  }

	
}

// While pre-generated interrupt handlers exist in stm32f0xx_it.h,
// They can be declared anywhere, so we will put them here!
void EXTI0_1_IRQHandler(void){
	GPIOC->ODR ^= (1<<8) | (1<<9); // Toggling PC8 and PC9 (green and orange LEDs)
	
	 // Delay loop of 1 second(1000ms)
    for (volatile uint32_t i = 0; i < 1000000; ++i) {
    }
	GPIOC->ODR ^= (1<<8) | (1<<9); // Toggling PC8 and PC9 (green and orange LEDs)
	
	// Clearing flag for input line 0 of EXTI Pending Register
	// Done to prevent looping. basically acknowledges its done, no longer pending
	EXTI->PR |= (1<<0); 
}




//-----------------------------------------------------------



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
