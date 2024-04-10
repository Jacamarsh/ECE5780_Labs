/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for lab4
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

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void TransmitCharacter(char character) {
    const uint32_t USART_TXE_FLAG = USART_ISR_TXE;

    // Wait for the transmit data register to be empty
    while (!(USART3->ISR & USART_TXE_FLAG)) {
        // Loop until register doesnt have a value
    }

    USART3->TDR = character; // send over the character
}

void CharReader(const char* message) {
    for (int i = 0; message[i] != '\0'; ++i) { // iterating looking for string
        TransmitCharacter(message[i]);
    }
}

char WaitForUSARTInput(void) {
    const uint32_t USART_RXNE_FLAG = USART_ISR_RXNE; // get non empty data reg

    // Wait for register to not be empty
    while (!(USART3->ISR & USART_RXNE_FLAG)) {
        // Loop until data present to read
    }

    return (char)(USART3->RDR); // Return the received character
}

void HandleLEDControl(char led, char action) {
    switch (led) {
        case 'o': // char for orange LED
        case 'r': // char for red LED
        case 'g': // char for green LED
        case 'b': // char for blue LED
            if (action == '0' || action == '1' || action == '2') {
                // LED logic placeholder
                // GPIOC->ODR modifier based on `led` and `action`
            } else {
                CharReader("illegal input: Use 0, 1, or 2.\n");
            }
            break;
        default:
            CharReader("illegal color: Use r, g, b, or o.\n");
            break;
    }
}

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    HAL_Init(); 

    while (1) {
        char firstInput = WaitForUSARTInput(); // getting 1st char
        char secondInput = WaitForUSARTInput(); // getting 2nd char

        HandleLEDControl(firstInput, secondInput); // handles input for LEDs

        HAL_Delay(10); // delay handler
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
