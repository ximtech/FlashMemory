/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include "SSD1306.h"
#include "FlashMemory.h"

int main(void) {


    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    initSSD1306(I2C1);

    initFlashMemory(FLASH_VOLTAGE_RANGE_3);
    setFlashMemorySector(FLASH_SECTOR_7);
    eraseSectorFlashMemory();

    writeToFlashMemory(0, FLASH_DATA_TYPE_BYTE, 122);
    writeToFlashMemory(1, FLASH_DATA_TYPE_BYTE, 11);
    writeToFlashMemory(2, FLASH_DATA_TYPE_BYTE, 56);

    uint8_t val1 = readFromFlashMemory(0, FLASH_DATA_TYPE_BYTE);
    uint8_t val2 = readFromFlashMemory(1, FLASH_DATA_TYPE_BYTE);
    uint8_t val3 = readFromFlashMemory(2, FLASH_DATA_TYPE_BYTE);

    printfStringSSD1306(0, 0, "U8:  %d %d %d", val1, val2, val3);

    writeToFlashMemory(0, FLASH_DATA_TYPE_HALF_WORD, 1222);
    writeToFlashMemory(2, FLASH_DATA_TYPE_HALF_WORD, 1111);
    writeToFlashMemory(3, FLASH_DATA_TYPE_HALF_WORD, 5600);

    uint16_t val4 = readFromFlashMemory(0, FLASH_DATA_TYPE_HALF_WORD);
    uint16_t val5 = readFromFlashMemory(2, FLASH_DATA_TYPE_HALF_WORD);
    uint16_t val6 = readFromFlashMemory(3, FLASH_DATA_TYPE_HALF_WORD);

    printfStringSSD1306(0, 8, "U16: %d %d %d", val4, val5, val6);

    writeToFlashMemory(0, FLASH_DATA_TYPE_WORD, 2221);
    writeToFlashMemory(2, FLASH_DATA_TYPE_WORD, 4444);
    writeToFlashMemory(3, FLASH_DATA_TYPE_WORD, 7890);

    uint32_t val7 = readFromFlashMemory(0, FLASH_DATA_TYPE_WORD);
    uint32_t val8 = readFromFlashMemory(2, FLASH_DATA_TYPE_WORD);
    uint32_t val9 = readFromFlashMemory(3, FLASH_DATA_TYPE_WORD);

    printfStringSSD1306(0, 16, "U32: %d %d %d", val7, val8, val9);

    writeToFlashMemory(0, FLASH_DATA_TYPE_DOUBLE_WORD, 33333333333333);
    uint64_t val10 = readFromFlashMemory(0, FLASH_DATA_TYPE_DOUBLE_WORD);
    printfStringSSD1306(0, 24, "U64: %llu", val10);

    LL_mDelay(3000);
    clearDisplaySSD1306();

    // Test floats and double
    writeFloatToFlashMemory(4, 12.2f);
    writeFloatToFlashMemory(5, 33.5f);
    writeFloatToFlashMemory(6, 5.4f);

    float val11 = readFloatFromFlashMemory(4);
    float val12 = readFloatFromFlashMemory(5);
    float val13 = readFloatFromFlashMemory(6);
    printfStringSSD1306(0, 0, "Floats: %.1f %.1f %.1f", val11, val12, val13);

    writeDoubleToFlashMemory(3, 3453456.23346);
    double val14 = readDoubleFromFlashMemory(3);
    printfStringSSD1306(0, 8, "Double: %.5f", val14);

    while (1) {
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_3) {
    }
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1) {

    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_12, 96, LL_RCC_PLLP_DIV_2);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {

    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {

    }
    LL_Init1msTick(100000000);
    LL_SetSystemCoreClock(100000000);
    LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    LL_I2C_InitTypeDef I2C_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    /**I2C1 GPIO Configuration
    PB6   ------> I2C1_SCL
    PB7   ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    /** I2C Initialization
    */
    LL_I2C_DisableOwnAddress2(I2C1);
    LL_I2C_DisableGeneralCall(I2C1);
    LL_I2C_EnableClockStretching(I2C1);
    I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
    I2C_InitStruct.ClockSpeed = 100000;
    I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
    I2C_InitStruct.OwnAddress1 = 0;
    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
    I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    LL_I2C_Init(I2C1, &I2C_InitStruct);
    LL_I2C_SetOwnAddress2(I2C1, 0);
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {

    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
