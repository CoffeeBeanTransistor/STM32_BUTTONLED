
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define DEBOUNCING_PERIOD 15
#define DELAY_TIME 1000



	typedef enum {
		BUTTON_RELEASED,
		BUTTON_PRESSED,
		BUTTON_RELEASED_DEBOUNCING,
		BUTTON_PRESSED_DEBOUNCING,
	} ButtonState;

	typedef enum {
	  LED_OFF,
	  LED_ON,
	} LedState;

	typedef struct LedButtonInfo LedButtonInfo;
	struct LedButtonInfo
	{
	  LedState currentLedState;
	  ButtonState previousButtonState;
	};

	typedef struct LedInfo LedInfo;
	struct LedInfo {
		LedState ledState;
		uint32_t previousTick;
	};

	typedef struct ButtonInfo ButtonInfo;
	struct ButtonInfo { 
		ButtonState buttonState;  //To keep the state of the button
		uint32_t prevTick;
		ButtonState state;    //The state for the state machine for button handler
	};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void handleButton(ButtonInfo *button);
ButtonState getButtonState(ButtonInfo *button);
void turnGreenLed(int state);
void doTapTurnOnTapTurnOffLed(LedButtonInfo *state, ButtonInfo *button);
void initButtonStateAndLedState(LedButtonInfo *ledState, ButtonInfo *buttonState);
void doBlinking(LedInfo *state);
void initLedBlinking(LedInfo *info);
void turnRedLed(int state);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void initLedBlinking(LedInfo *info) {
	info->ledState = LED_OFF;
	info->previousTick = HAL_GetTick();
}

void initButtonStateAndLedState(LedButtonInfo *ledState, ButtonInfo *buttonState) {
	buttonState->buttonState= HAL_GPIO_ReadPin(userButton_GPIO_Port,userButton_Pin);
	buttonState->prevTick = 0;
	buttonState->state = BUTTON_RELEASED;
	ledState->currentLedState = LED_OFF;
	ledState->previousButtonState = BUTTON_RELEASED;
}

ButtonState getButtonState(ButtonInfo *button) {
		return button->buttonState;
	}

void doBlinking(LedInfo *state) {
	switch(state->ledState) {
	case LED_OFF :
		if(HAL_GetTick() > state->previousTick + DELAY_TIME) {
			turnRedLed(LED_ON);
			state->ledState = LED_ON;
			state->previousTick = HAL_GetTick();
		}
	break;

	case LED_ON  :
		if(HAL_GetTick() > state->previousTick + DELAY_TIME) {
			turnRedLed(LED_OFF);
			state->ledState = LED_OFF;
			state->previousTick = HAL_GetTick();
		}
	break;

	}
}


void handleButton(ButtonInfo *button) {
		switch(button->state) {
			case BUTTON_RELEASED :
				if(HAL_GPIO_ReadPin(userButton_GPIO_Port,userButton_Pin) == GPIO_PIN_SET) {
					button->prevTick = HAL_GetTick();
					button->state = BUTTON_PRESSED_DEBOUNCING;
				}
			break;

			case BUTTON_PRESSED_DEBOUNCING :
				if(HAL_GetTick() > button->prevTick + DEBOUNCING_PERIOD) {
					if(HAL_GPIO_ReadPin(userButton_GPIO_Port,userButton_Pin == GPIO_PIN_SET)) {
						button->buttonState = BUTTON_PRESSED;
						button->state = BUTTON_PRESSED;
					}
				}
			break;

			case BUTTON_PRESSED :
				if(HAL_GPIO_ReadPin(userButton_GPIO_Port,userButton_Pin) == GPIO_PIN_RESET) {
						button->prevTick = HAL_GetTick();
						button->state = BUTTON_RELEASED_DEBOUNCING;

				}
			break;

			case BUTTON_RELEASED_DEBOUNCING :
				if(HAL_GetTick() > button->prevTick + DEBOUNCING_PERIOD) {
					if(HAL_GPIO_ReadPin(userButton_GPIO_Port,userButton_Pin) == GPIO_PIN_RESET) {
						button->buttonState = BUTTON_RELEASED;
						button->state = BUTTON_RELEASED;
					}
				}
			break;
		}
	}

void turnGreenLed(int state) {

		if(state == LED_ON)
			HAL_GPIO_WritePin(greenLED_GPIO_Port, greenLED_Pin, GPIO_PIN_SET);

		else if (state == LED_OFF)
			HAL_GPIO_WritePin(greenLED_GPIO_Port, greenLED_Pin, GPIO_PIN_RESET);

}

void turnRedLed(int state) {

		if(state == LED_ON)
			HAL_GPIO_WritePin(redLED_GPIO_Port, redLED_Pin, GPIO_PIN_SET);

		else if (state == LED_OFF)
			HAL_GPIO_WritePin(redLED_GPIO_Port, redLED_Pin, GPIO_PIN_RESET);

}

void doTapTurnOnTapTurnOffLed(LedButtonInfo *state, ButtonInfo *button)
{
	int prev_button = state->previousButtonState;

		  if(state->currentLedState == LED_OFF)
		  {
		    switch(prev_button)
		    {
		      case BUTTON_RELEASED:
		        if(getButtonState(button) == BUTTON_RELEASED)
		        {
		          state->currentLedState = LED_OFF;
		          state->previousButtonState = BUTTON_RELEASED;
		        }
		        else
		        {
		          turnGreenLed(LED_ON);
		          state->currentLedState = LED_OFF;
		          state->previousButtonState = BUTTON_PRESSED;
		        }
		        break;

		      case BUTTON_PRESSED:
		        if(getButtonState(button) == BUTTON_RELEASED)
		        {
		          state->currentLedState = LED_ON;
		          state->previousButtonState = BUTTON_RELEASED;
		        }
		        else
		        {
		          state->currentLedState = LED_OFF;
		          state->previousButtonState = BUTTON_PRESSED;
		        }
		        break;
		    }
		  }

		  else if(state->currentLedState == LED_ON)
		  {
		    switch(prev_button)
		    {
		      case BUTTON_RELEASED:
		        if(getButtonState(button) == BUTTON_RELEASED)
		        {
		          state->currentLedState = LED_ON;
		          state->previousButtonState = BUTTON_RELEASED;
		        }
		        else
		        {
		          state->currentLedState = LED_ON;
		          state->previousButtonState = BUTTON_PRESSED;
		        }
		        break;

		      case BUTTON_PRESSED:
		        if(getButtonState(button) == BUTTON_RELEASED)
		        {
		          turnGreenLed(LED_OFF);
		          state->currentLedState = LED_OFF;
		          state->previousButtonState = BUTTON_RELEASED;
		        }
		        else
		        {
		          state->currentLedState = LED_ON;
		          state->previousButtonState = BUTTON_PRESSED;
		        }
		        break;
		    }
		  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	LedButtonInfo state;
	ButtonInfo button;
	LedInfo info;

	initButtonStateAndLedState(&state, &button);
	initLedBlinking(&info);

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 doTapTurnOnTapTurnOffLed(&state, &button);
	 handleButton(&button);
	 doBlinking(&info);

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, greenLED_Pin|redLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : userButton_Pin */
  GPIO_InitStruct.Pin = userButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(userButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : greenLED_Pin redLED_Pin */
  GPIO_InitStruct.Pin = greenLED_Pin|redLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
