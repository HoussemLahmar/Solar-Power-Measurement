/* SPDX-License-Identifier: BSD-2-Clause
 *
 * Description: Solar Panel Monitoring System
 *
 * This file is part of a project developed for monitoring solar panel performance using an STM32 microcontroller. The system measures various parameters including voltage, current, temperature, and light intensity to assess the efficiency and health of the solar panel system.
 *
 * Features:
 * - Utilizes STM32 microcontroller for data acquisition and control.
 * - Reads voltage and current data from solar panels using ADC channels.
 * - Integrates additional sensors for monitoring temperature and light intensity.
 * - Implements LCD display for real-time visualization of data.
 * - Implements GPIO control for system status indication and control.
 *
 * Copyright (c) 2024, Houssem-eddine Lahmer <Houssemeddine.lahmar@etudiant-enit.utm.tn>
 * All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);

/* Private user code ---------------------------------------------------------*/
uint32_t adcRaw[4];

float R1 = 30000.0;
float R2 = 7500.0;
float RShunt = 100.0;

int lux;
float voltage, shuntVol, current, temp;

////////////////////////////////////////////////////////////////////LCD CODE

#define SLAVE_ADDRESS_LCD 0x3F << 1

int convInt(uint32_t dump) {
    char data[100];
    sprintf(data, "%lu", dump);
    int dataInt;
    sscanf(data, "%d", &dataInt);
    return dataInt;
}

void lcd_send_cmd(char cmd) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C;
    data_t[1] = data_u | 0x08;
    data_t[2] = data_l | 0x0C;
    data_t[3] = data_l | 0x08;
    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *)data_t, 4, 100);
}

void lcd_send_data(char data) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D;
    data_t[1] = data_u | 0x09;
    data_t[2] = data_l | 0x0D;
    data_t[3] = data_l | 0x09;
    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *)data_t, 4, 100);
}

void lcd_init(void) {
    // 4 bit initialization
    HAL_Delay(50); // wait for >40ms
    lcd_send_cmd(0x30);
    HAL_Delay(5); // wait for >4.1ms
    lcd_send_cmd(0x30);
    HAL_Delay(1); // wait for >100us
    lcd_send_cmd(0x30);
    HAL_Delay(10);
    lcd_send_cmd(0x20); // 4bit mode
    HAL_Delay(10);

    // display initialization
    lcd_send_cmd(0x20); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
    HAL_Delay(1);
    lcd_send_cmd(0x08); // Display on/off control --> D=0,C=0, B=0  ---> display off
    HAL_Delay(1);
    lcd_send_cmd(0x01); // clear display
    HAL_Delay(1);
    HAL_Delay(1);
    lcd_send_cmd(0x06); // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
    HAL_Delay(1);
    lcd_send_cmd(0x0C); // Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_clear(void) {
    lcd_send_cmd(0x80);
    for (int i = 0; i < 70; i++) {
        lcd_send_data(' ');
    }
}

void lcd_put_cur(int row, int col) {
    switch (row) {
    case 0:
        col |= 0x80;
        break;
    case 1:
        col |= 0xC0;
        break;
    }
    lcd_send_cmd(col);
}

void lcd_send_string(char *str) {
    while (*str)
        lcd_send_data(*str++);
}

void showLCD(char dump[100], int col, int row, bool cls) {
    if (cls) {
        lcd_clear();
    }
    lcd_put_cur(row, col);
    lcd_send_string(dump);
}
//////////////////////////////////////////////////////////////////////////////////////////
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start_DMA(&hadc1, adcRaw, 4);

    lcd_init(); /// LCD INITIAISE
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        char voltageLCD[30];
        char currentLCD[30];
        char tempLCD[30];
        char luxLCD[30]; // LCD CHARACTER VARIABLES

        int voltageADC = convInt(adcRaw[0]);
        int shuntVolADC = convInt(adcRaw[1]);
        int tempADC = convInt(adcRaw[2]);
        int luxADC = convInt(adcRaw[3]);

        voltage = voltageADC * (5.0 / 4095.0) * ((R1 + R2) / R2);
        shuntVol = shuntVolADC * (5.0 / 4095.0) * ((R1 + R2) / R2);

        current = (shuntVol / RShunt) * 1000;
        temp = ((tempADC * (5000 / 4095.0)) / 10) - 10;

        lux = (luxADC / 4095.0) * 100;

        sprintf(voltageLCD, "V:%.1fV", voltage);
        showLCD(voltageLCD, 0, 0, true);

        sprintf(currentLCD, "I:%.1fmA", current);
        showLCD(currentLCD, 0, 1, false);

        sprintf(tempLCD, "T:%.1fC", temp);
        showLCD(tempLCD, 8, 0, false);

        sprintf(luxLCD, "L:%dLx", lux);
        showLCD(luxLCD, 9, 1, false);

        if (temp > 60) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        }

        HAL_Delay(1000);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void) {
    /* Function body */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {
    /* Function body */
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void) {
    /* Function body */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {
    /* Function body */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) {
    /* Function body */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    /* Function body */
}

/* USER CODE BEGIN 4 */
/////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* Function body */
    __disable_irq();
    while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
    /* Function body */
}
#endif /* USE_FULL_ASSERT */
