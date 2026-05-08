/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>   // Reikalinga sprintf funkcijai
#include <string.h>  // Reikalinga tekstinems operacijoms
#include <math.h>
#include "I2C_LCD.h"
#include <arm_math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t id;      
    uint32_t pin_code;       
    uint8_t mic_gain_mode;   // 0: 40dB, 1: 50dB, 2: 60dB
} DeviceConfig;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MyI2C_LCD I2C_LCD_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t freq_index = 0; 
char msg[64];
  
volatile uint64_t rms_sum = 0;
volatile uint32_t rms_count = 0;
  
float last_freq = 0;
float last_rms = 0;

const int16_t MID_POINT = 2048; 
const int16_t THRESHOLD_Q15 = 2500; // 2500/(2^4)=156 in ADC scale

// Filter parameters
#define NUM_STAGES 1
arm_biquad_casd_df1_inst_q15 S_q15;
q15_t biquad_state_q15[4 * NUM_STAGES];
q15_t filtered_val_q15;

// Coefficients in Q15 format [b0, 0, b1, b2, a1, a2]
q15_t biquad_coeffs_q15[6] = {
	8063, 0, 0, -8063, 16127, -257
};


// TIM2 parameters for measuring frequency
volatile uint32_t last_capture_tick = 0;
volatile uint32_t period_in_ticks = 0;
volatile uint8_t new_period_flag = 0;

// Configuration
DeviceConfig currentConfig;
uint8_t config_mode = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define FLASH_CONFIG_ADDR 0x0800F800 

void SaveConfig(void) {
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError;

    HAL_FLASH_Unlock();
    
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_CONFIG_ADDR;
    eraseInit.NbPages = 1;
    
    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) == HAL_OK) {
        uint32_t *dataPtr = (uint32_t*)&currentConfig;
        for (uint32_t i = 0; i < (sizeof(DeviceConfig) / 4); i++) {
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CONFIG_ADDR + (i * 4), dataPtr[i]);
				}
    }
    HAL_FLASH_Lock();
}
void LoadConfig(void) {
    memcpy(&currentConfig, (uint32_t*)FLASH_CONFIG_ADDR, sizeof(DeviceConfig));
    
    if (currentConfig.id != 0xABABABAB) {
        currentConfig.id = 0xABABABAB;
        currentConfig.pin_code = 0000;       
        currentConfig.mic_gain_mode = 2;     
        SaveConfig();
    }
}

uint32_t UART_Read(void) {
    char buffer[16];
    uint8_t i = 0;
    uint8_t ch;
    
    while (1) {
        if (HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY) == HAL_OK) {
            
            if (ch == '\r' || ch == '\n') {
                if (i == 0) continue; 
                buffer[i] = '\0';
                printf("\r\n"); 
                break;
            } 
            
            else if (ch == '\b' || ch == 127) {
                if (i > 0) {
                    i--;
                    printf("\b \b");
                }
            }
            else if (ch >= '0' && ch <= '9' && i < 15) buffer[i++] = ch;
        }
    }
    return atol(buffer); 
}

void Set_Mic_Gain(uint8_t mode) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = Mic_Gain_Pin;

    if (mode == 0) { // 40dB - Vdd
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(Mic_Gain_GPIO_Port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(Mic_Gain_GPIO_Port, Mic_Gain_Pin, GPIO_PIN_SET);
    } 
    else if (mode == 1) { // 50dB - GND
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(Mic_Gain_GPIO_Port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(Mic_Gain_GPIO_Port, Mic_Gain_Pin, GPIO_PIN_RESET);
    } 
    else { // 60dB - HiZ
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(Mic_Gain_GPIO_Port, &GPIO_InitStruct);
    }
}
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
  MX_TIM6_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	arm_biquad_cascade_df1_init_q15(&S_q15, NUM_STAGES, biquad_coeffs_q15, biquad_state_q15, 0);
	
  I2C_LCD_Init(MyI2C_LCD);
  I2C_LCD_Clear(MyI2C_LCD);
	
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start(&hadc);
	
	uint32_t last_tick_uart = HAL_GetTick();
  uint32_t last_tick_lcd = HAL_GetTick();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LoadConfig(); 
	Set_Mic_Gain(currentConfig.mic_gain_mode);

  while (1)
  {
    uint8_t rec_data;
    if (HAL_UART_Receive(&huart2, &rec_data, 1, 10) == HAL_OK) {
        
        if (config_mode == 0 && rec_data == 'a') { 
            printf("\r\nIveskite PIN: ");
            int16_t entered_pin = 0;
						entered_pin = UART_Read();
            
            if (entered_pin == currentConfig.pin_code) {
                config_mode = 1;
							
								I2C_LCD_Clear(MyI2C_LCD);
                I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
                I2C_LCD_WriteString(MyI2C_LCD, "Vyksta");
                I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
                I2C_LCD_WriteString(MyI2C_LCD, "Konfiguravimas");
							
								printf("\r\nKonfiguravimo meniu:\r\n");
                printf("1. Keisti stiprinima (Dabar: %d)\r\n", currentConfig.mic_gain_mode);
                printf("2. Keisti PIN\r\n");
                printf("s. Issaugoti ir Iseiti\r\n");
            } else {
                printf("\r\nNeteisingas PIN\r\n");
            }
        }
        
        else if (config_mode == 1) {
            if (rec_data == '1') {
                printf("\r\nPasirinkite Stiprinima (0:40dB, 1:50dB, 2:60dB): ");
                int16_t gain;
								gain = UART_Read();
                currentConfig.mic_gain_mode = (uint8_t)gain;
                Set_Mic_Gain(currentConfig.mic_gain_mode);
                printf("Atnaujinta!\r\n");
            }
            else if (rec_data == '2') {
                printf("\r\nIveskite nauja PIN: ");
								currentConfig.pin_code = UART_Read();
                printf("PIN pakeistas\r\n");
            }
            else if (rec_data == 's') {
                SaveConfig(); 
                config_mode = 0;
								I2C_LCD_Clear(MyI2C_LCD);
                printf("\r\nIssaugota.\r\n");
            }
        }
    }
    uint32_t current_tick = HAL_GetTick();

    // Display in terminal evrery 0.5s
    if (config_mode == 0 && current_tick - last_tick_uart >= 500) 
    {
			if (rms_count > 0)
        {
            
            // RMS voltage calculation
            double mean_sq = (double)rms_sum / (double)rms_count;
            last_rms = (sqrt(mean_sq) / 32768.0f) * 3.3f;
						
						// Frequency calculation
						if (new_period_flag) 
							{
									last_freq = 1000000.0f / (float)period_in_ticks;
									new_period_flag = 0; 
							} 
						else 
							{
									// If no frequency was recorded
									last_freq = 0;
							}
				
            sprintf(msg, "Urms: %.3f V ; F: %.1f Hz\r\n", last_rms, last_freq);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
        }

        rms_count = 0;
        rms_sum = 0;
        last_tick_uart = current_tick;
    }

    // Display in LCD every 2s
    if (config_mode == 0 && current_tick - last_tick_lcd >= 2000) 
    {
        char line1[18], line2[18];
        
        sprintf(line1, "Urms: %.3f V    ", last_rms);  
				sprintf(line2, "F: %.1f Hz   ", last_freq);
        I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
        I2C_LCD_WriteString(MyI2C_LCD, line1);
        I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
        I2C_LCD_WriteString(MyI2C_LCD, line2);

        last_tick_lcd = current_tick;
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Mic_Gain_GPIO_Port, Mic_Gain_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, COM4_Pin|COM3_Pin|COM2_Pin|COM1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Mic_Gain_Pin */
  GPIO_InitStruct.Pin = Mic_Gain_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Mic_Gain_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COM4_Pin COM3_Pin COM2_Pin COM1_Pin */
  GPIO_InitStruct.Pin = COM4_Pin|COM3_Pin|COM2_Pin|COM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		if (htim->Instance == TIM6)
    {
        uint16_t raw_adc = HAL_ADC_GetValue(&hadc);

        // Conversion for Q15 after removing DC bias
        q15_t input_q15 = (q15_t)((int16_t)raw_adc - 2048) << 4;

        // Filter
        arm_biquad_cascade_df1_q15(&S_q15, &input_q15, &filtered_val_q15, 1);

        // RMS voltage calculation
        rms_sum += (int64_t)filtered_val_q15 * filtered_val_q15;
        rms_count++;
        
			// Frequency measurement
				if (freq_index == 0 && filtered_val_q15 > THRESHOLD_Q15) 
        {
            uint32_t current_tick = __HAL_TIM_GET_COUNTER(&htim2);
            
            if (last_capture_tick > 0) {
                period_in_ticks = current_tick - last_capture_tick;
                new_period_flag = 1; 
            }
            
            last_capture_tick = current_tick;
            freq_index = 1;
        } 
        else if (freq_index == 1 && filtered_val_q15 < -THRESHOLD_Q15) 
        {
            freq_index = 0; 
        }

        HAL_ADC_Start(&hadc);
    }
		
	}
	
	int fputc(int ch, FILE *f) {
		HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
		return ch;
	}
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
