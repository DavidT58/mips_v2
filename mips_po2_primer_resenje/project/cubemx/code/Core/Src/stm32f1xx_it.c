/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */

	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles TIM1 update interrupt.
 */
void TIM1_UP_IRQHandler(void)
{
	/* USER CODE BEGIN TIM1_UP_IRQn 0 */

	/* USER CODE END TIM1_UP_IRQn 0 */
	HAL_TIM_IRQHandler(&htim1);
	/* USER CODE BEGIN TIM1_UP_IRQn 1 */

	/* USER CODE END TIM1_UP_IRQn 1 */
}

/**
 * @brief This function handles TIM1 capture compare interrupt.
 */
void TIM1_CC_IRQHandler(void)
{
	/* USER CODE BEGIN TIM1_CC_IRQn 0 */

	/* USER CODE END TIM1_CC_IRQn 0 */
	HAL_TIM_IRQHandler(&htim1);
	/* USER CODE BEGIN TIM1_CC_IRQn 1 */

	/* USER CODE END TIM1_CC_IRQn 1 */
}

/**
 * @brief This function handles TIM3 global interrupt.
 */
void TIM3_IRQHandler(void)
{
	/* USER CODE BEGIN TIM3_IRQn 0 */

	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_IRQn 1 */

	/* USER CODE END TIM3_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI15_10_IRQn 0 */

	/* USER CODE END EXTI15_10_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
	/* USER CODE BEGIN EXTI15_10_IRQn 1 */

	/* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

volatile uint32_t gu32_Freq = 0;
volatile uint8_t prikazFreq = 0;
extern uint8_t cifre[];

extern uint32_t counter;
extern uint32_t sekunde;
extern uint32_t minuti;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_11)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		counter = 0;
		sekunde = 0;
		minuti = 0;
	}

	if (GPIO_Pin == GPIO_PIN_10)
	{
		cifre[3] = gu32_Freq % 10;
		cifre[2] = (gu32_Freq / 10) % 10;
		cifre[1] = (gu32_Freq / 100) % 10;
		cifre[0] = (gu32_Freq / 1000) % 10;
		prikazFreq ^= 1;
	}
}

#define PRVA_UZLAZNA   0
#define DRUGA_UZLAZNA  1
#define F_CLK  8000000 // 8000 * 1000 (da bi bilo u mHz)

volatile uint8_t gu8_State = PRVA_UZLAZNA;
volatile uint32_t gu32_T1 = 0;
volatile uint32_t gu32_T2 = 0;
volatile uint32_t gu32_Ticks = 0;
volatile uint16_t gu16_TIM1_OVC = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (gu8_State == PRVA_UZLAZNA)
	{
		gu32_T1 = TIM1->CCR1;
		gu16_TIM1_OVC = 0;
		gu8_State = DRUGA_UZLAZNA;
	}
	else if (gu8_State == DRUGA_UZLAZNA)
	{
		gu32_T2 = TIM1->CCR1;
		gu32_Ticks = (gu32_T2 + (gu16_TIM1_OVC * 79)) - gu32_T1;
		gu32_Freq = (uint32_t) (F_CLK / gu32_Ticks); //  za 10ms naprave se 80 prekida
		gu8_State = PRVA_UZLAZNA;
	}
}

uint8_t ON_OFF = 0;
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (ON_OFF == 0)
	{
		htim->Instance->CCR1 = 11999;
	}
	else
	{
		htim->Instance->CCR1 = 3999;
	}
	htim->Instance->CNT = 0;
	ON_OFF ^= 1;
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
