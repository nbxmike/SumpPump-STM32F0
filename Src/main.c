/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 * Copyright (c) 2019 Michael McCormack.
 * All rights reserved.
 *
 * This software component is licensed by Michael McCOrmack under BSD
 * 3-Clause license, the "License"; You may not use this file except in
 * compliance with the License.
 *
 * BSD 3-Clause license:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 * @details The function of this software is to control a sump pump which
 * removes water in the basement of my house.  As my house is built on granite
 * slab, there is not sufficient room to sink a normal 24" / 60cm deep sump
 * basin and commercial pump.  I have resorted to using a garden fountain
 * pump, a solid state relay, a STM32F0 discovery board, and, a set of level
 * sensors together with this firmware.
 *
 * Four sensors are used with this project at the moment which are two
 * pieces of the following hardware:
 *  1) Floats - these are mechanical devices that are moved by water and
 *     supply a switch closure.  Firmware assumes that these switches close
 *     to ground and provides a pullup on the input.
 *  2) CapSense - two electrodes are fixed in the Z dimension and when water
 *     covers them the capacitance between them changes.  These sensors used
 *     at time of publication provide push-pull output so no pullup is
 *     required, however, the code does provide one so that in the future
 *     if an open collector version is used the change might be simpler.
 *
 * Two pieces of each type of sensor are used, one of each type for a high
 * water turn on sensor and one for low level turn off.  The use of two types
 * of sensors was due to my skepticism about the reliability of these devices.
 *
 * While it is completely unnecessary for this project, I have chosen to use
 * FreeRTOS for this project.  I do a lot of work with it in my day job and
 * wanted to see how easy / hard it is to use it with the ST development IDE.
 * So far, all I can say is the obfuscation on the FreeRTOS system under a
 * CMSIS wrapper is not a feature, at least for me.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define floatLevelHigh        ( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) )
#define floatLevelLow         ( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) )
#define capSenseHigh          ( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) )
#define capSenseLow           ( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)  )

#define FLOAT_UP              GPIO_PIN_SET
#define FLOAT_DOWN            GPIO_PIN_RESET
#define CAPSENSE_UP           GPIO_PIN_RESET
#define CAPSENSE_DOWN         GPIO_PIN_SET

#define sumpMotorControl(X)   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, X)
#define PUMP_ON               GPIO_PIN_RESET
#define PUMP_OFF              GPIO_PIN_SET

#define greenLEDControl(X)    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, X)
#define blueLEDControl(X)     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, X)
#define LED_ON                GPIO_PIN_RESET
#define LED_OFF               GPIO_PIN_SET

// Values used to decide when to turn on the pump, rectally derived
#define MAX_WATER_COUNT 2000
#define ON_WATER_COUNT  1999
#define OFF_WATER_COUNT 1000

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId sumpPumpTaskHandle;
/* USER CODE BEGIN PV */
static int levelByFLoat = 0;                    // FLoat style level sensor counter for debouncing
static int levelByCapacatance = 0;              // Capacitive fork sensor counter for debouncing
static GPIO_PinState pumpOn = PUMP_OFF;         // Pump SSR control signal as a variable for debugging
static int blueLEDOn = 0;                       // Blue LED "
static int greenLEDOn = 0;                      // Green LED "
static int ledCount = 0;                        // Counter to use for LED blink rate
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);
void sumpTask(void const * argument);

/* USER CODE BEGIN PFP */

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

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of sumpPumpTask */
  osThreadDef(sumpPumpTask, sumpTask, osPriorityNormal, 0, 128);
  sumpPumpTaskHandle = osThreadCreate(osThread(sumpPumpTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief  Reads the value of the level detectors.
 * @fn ReadWaterLevel( void )
 * @param  argument: Not used
 * @retval None
 * @details The ReadWaterLevel function reads four GPIO pins and debounces
 * them using a set of up / down counters implemented in software.
 */
void ReadWaterLevel(void)
{
  int flh,fll,csh,csl;
  flh = floatLevelHigh;
  fll = floatLevelLow;
  csh = capSenseHigh;
  csl = capSenseLow;

  if ((FLOAT_UP == floatLevelHigh) && (FLOAT_UP == floatLevelLow))
  {
    ++levelByFLoat;
  }
  else if (FLOAT_UP != floatLevelLow)
  {
    --levelByFLoat;
  }

  if ((CAPSENSE_UP == capSenseHigh) && (CAPSENSE_UP == capSenseLow))
  {
    ++levelByCapacatance;
  }
  else if (CAPSENSE_UP != capSenseLow)
  {
    --levelByCapacatance;
  }
  return;
}

/**
 * @brief  Determines if the pump should be on or off
 * @fn EvaluateLevelCounts( void )
 * @param  argument: Not used
 * @retval sump pump control in GPIO_PinState
 * @details The EvaluateLevelCounts function debounces and range limits the
 * counters built from the inputs.
 */
GPIO_PinState EvaluateLevelCounts(void)
{
  GPIO_PinState pumpState;

  // First bound the counters
  if (0 > levelByFLoat)
  {
    levelByFLoat = 0;
  }
  if (MAX_WATER_COUNT < levelByFLoat)
  {
    levelByFLoat = MAX_WATER_COUNT;
  }

  if (0 > levelByCapacatance)
  {
    levelByCapacatance = 0;
  }
  if (MAX_WATER_COUNT < levelByCapacatance)
  {
    levelByCapacatance = MAX_WATER_COUNT;
  }

  // Use the counters to decide if the pump should be on or off
  if ((ON_WATER_COUNT < levelByFLoat) && (ON_WATER_COUNT < levelByCapacatance))
  {
    pumpState = PUMP_ON;
  }
  if ((OFF_WATER_COUNT > levelByFLoat) || (OFF_WATER_COUNT > levelByCapacatance))
  {
    pumpState = PUMP_OFF;
  }

  return(pumpState);
}

/**
 * @brief Outputs system info on available LEDs, often changes during debug
 * @fn userInterface( void )
 * @param  argument: Not used
 * @retval sump pump control in GPIO_PinState
 * @details The userInterface function does its best to signal the software state
 * using the available LEDs.  It presently sends the motor SSR control signal
 * on the blue LED and just a "running" blink on the green LED.
 */
void userInterface(void)
{

  ++ledCount;
  if (512 <= ledCount)
  {
    ledCount = 0;
  }
  if (ledCount & 0x20)
  {
    greenLEDControl(LED_ON);
  }
  else
  {
    greenLEDControl(LED_OFF);
  }
  if (  ((0 != pumpOn) && (ledCount & 0x10))
     || ((0 == pumpOn) && (ledCount & 0x40)))
  {
    blueLEDControl(LED_ON);
  }
  else
  {
    blueLEDControl(LED_OFF);
  }

  return;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
    
    
    

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_sumpTask */
/**
 * @brief Function implementing the sumpPumpTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_sumpTask */
void sumpTask(void const * argument)
{
  /* USER CODE BEGIN sumpTask */

  greenLEDControl(LED_ON);

  /* Infinite loop */
  for (;;)
  {

    ReadWaterLevel();

    pumpOn = EvaluateLevelCounts();

    sumpMotorControl(pumpOn);

    userInterface();

    osDelay(5);
  }
  /* USER CODE END sumpTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
