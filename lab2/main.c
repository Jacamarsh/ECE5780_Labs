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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
     HAL_Init();
     __HAL_RCC_GPIOC_CLK_ENABLE(); // clock C enabled

     __HAL_RCC_GPIOA_CLK_ENABLE(); // clock A enabled

     // configuring GPIO pins 6, 7, 8, and 9 on port c (GPIO port c is called GPIOC)
     // (1<<(1st GPIO pin number)*2) | (1<<( 2nd GPIO pin number)*2) etc --> notation to set pins to output mode
     // using MODER (GPIO port mode register), setting mode to 1 categorizes pins to be in output mode
     GPIOC->MODER |= (1<<6*2) | (1<<7*2) | (1<<8*2) | (1<<9*2);

    // SystemClock_Config();
     // MODER with mode to 0 --> input mode
     GPIOA->MODER |= (0<<(0*2)); //  makes pin 0 (for port A) an input

     // PUPDR: GPIO pull-up/pull-down resister. 1<<0: shifts 1 to left by 0 positions (means unchanged). 
     // GPIOA->MODER |= (0<<(0*2)); //  makes pin 0 (for port A) an input
     // (1<<0) --> mode/value 1, pin 0: enable pull-up for pin 0
     GPIOA->PUPDR |= (1<<0); 

     EXTI->IMR |= (1<<0); // interrupt mask register enabled (from line 0)
     EXTI->RTSR |= (1<<0); // sets up external interrupt on the rising edge (transition from low to high voltage --> triggers EXTI0 interrupt)

     // RCC: reset and clock control, APB2ENR: APB2 peripheral clock enable register (used to enable/disable the clock to such peripherals)
     // RCC_APB2ENR_SYSCFGCOMPEN: bit mask to sys config controller (SYSCFG).
     // Takes bits from APB2 clk en register, does bitwise OR w/ value from bitmask of sys config controller, and writes result to APb2 clk en register.
     // Done to set SYSCFG/COMP clock enable bit to 1 while not changing any other bits
     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

     SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // configs system copntroller to route external interrupt from pin 0 on GPIO port A (PA0) to the EXTI line 0.
     // Done by seeting the bits in one of exernal interrupt config registers (EXTICR)


     NVIC_EnableIRQ(EXTI0_1_IRQn); // external interrupt enabled for pin 0
     NVIC_SetPriority(EXTI0_1_IRQn, 3); // pin 0 priority set to 3
     NVIC_SetPriority(SysTick_IRQn, 2); // sysTick priority set to 2

     while (1)
     {
       HAL_Delay(400);
       GPIOC->ODR ^= (1<<6);
         // coninuous loop, any interrupt enables GPIO pins!
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
