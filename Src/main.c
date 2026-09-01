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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mfrc522.h"
#include "VL53L0X.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MI_OK 0 

/*
 * SERVO NOTE:
 * This motor behaves as a CONTINUOUS-ROTATION servo, not a standard positional
 * SG90. For this type: ~1500us = stopped, <1500us = spin one direction,
 * >1500us = spin the other direction. The further from 1500, the faster it spins.
 *
 * SERVO_FORWARD / SERVO_REVERSE below are starting guesses. If the motor doesn't
 * turn roughly 90 degrees in SERVO_MOVE_TIME_MS, nudge these values closer to or
 * further from 1500 (closer = slower turn, further = faster turn) until the
 * physical rotation matches ~90 degrees in 2 seconds. Do not change
 * SERVO_MOVE_TIME_MS to compensate; tune the pulse widths instead.
 */
#define SERVO_STOP           1500  // Pulse width that holds the motor still
#define SERVO_FORWARD        1700  // Pulse width driving it "open" direction
#define SERVO_REVERSE        1300  // Pulse width driving it "close" direction
#define SERVO_MOVE_TIME_MS   2000  // Time spent moving each direction
#define SERVO_HOLD_TIME_MS   3000  // Time held open before returning
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t rfid_id[5];
char uid_string[20];
uint16_t user_distance = 9999;
struct VL53L0X my_tof; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void ESP8266_Send_User_Post(char* scanned_uid);
void ESP8266_Send_Log_Post(char* scanned_uid, const char* status);
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
  // Force the alert LED OFF immediately (Active-HIGH means RESET = OFF)
  HAL_GPIO_WritePin(GPIOB, LED_ALERT_Pin, GPIO_PIN_RESET);
  
  // 2-second initial boot delay, allowing the vibration sensor comparator to stabilize
  HAL_Delay(2000); 

  // Start the PWM signal for the continuous-rotation motor on PA1 (TIM2_CH2)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_STOP); // Motor stopped at boot

  // 1. Configure and Initialize the Laser Ranging Sensor
  my_tof.io_2v8 = true;
  my_tof.address = 0x52; 
  my_tof.io_timeout = 500;
  VL53L0X_init(&my_tof); 

  // 2. Set system state to "OFF" using PC13
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  // 3. Standby Loop: Trap the system here until a user is within 200 mm
  while (1) {
      
      // Check for vibration tampering while asleep (Trigger on SET/HIGH)
      if (HAL_GPIO_ReadPin(GPIOB, VIB_SENSOR_Pin) == GPIO_PIN_SET) {
          HAL_GPIO_WritePin(GPIOB, LED_ALERT_Pin, GPIO_PIN_SET);   // Turn LED ON
      } else {
          HAL_GPIO_WritePin(GPIOB, LED_ALERT_Pin, GPIO_PIN_RESET); // Keep LED OFF
      }

      user_distance = VL53L0X_readRangeSingleMillimeters(&my_tof); 
      
      if (user_distance > 0 && user_distance < 200) {
          break; 
      }
      HAL_Delay(500); 
  }

  // 4. System Woke Up! Turn PC13 LED "ON" 
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  // 5. Connect to Wi-Fi now that the system is awake
  char wifiCmd[] = "AT+CWJAP=\"Rafi\",\"rafirabi\"\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)wifiCmd, strlen(wifiCmd), 5000);
  HAL_Delay(5000); 
  
  // 6. Initialize the RFID scanner
  MFRC522_Init();
  
  // 7. Initialize timeout tracking variables for the main loop
  uint32_t last_activity_time = HAL_GetTick();
  const uint32_t TIMEOUT_THRESHOLD = 15000; // 15 seconds

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // 1. Check for vibration tampering while awake (Trigger on SET/HIGH)
    if (HAL_GPIO_ReadPin(GPIOB, VIB_SENSOR_Pin) == GPIO_PIN_SET) {
        HAL_GPIO_WritePin(GPIOB, LED_ALERT_Pin, GPIO_PIN_SET);   // Turn LED ON
    } else {
        HAL_GPIO_WritePin(GPIOB, LED_ALERT_Pin, GPIO_PIN_RESET); // Keep LED OFF
    }

    // 2. Check ToF Sensor to see if user is still standing there
    user_distance = VL53L0X_readRangeSingleMillimeters(&my_tof);
    if (user_distance > 0 && user_distance < 50) {
        last_activity_time = HAL_GetTick(); // Reset inactivity timer
    }

    // 3. Detect RFID card
    if (MFRC522_Request(0x26, rfid_id) == MI_OK) {
        
        if (MFRC522_Anticoll(rfid_id) == MI_OK) {
            
            sprintf(uid_string, "%02X%02X%02X%02X", rfid_id[0], rfid_id[1], rfid_id[2], rfid_id[3]);
            
            // Send the POST request to the /api/logs endpoint
            ESP8266_Send_Log_Post(uid_string, "granted");
            
            // Drive motor "open" direction for SERVO_MOVE_TIME_MS, then stop
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_FORWARD);
            HAL_Delay(SERVO_MOVE_TIME_MS);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_STOP);

            // Hold open for SERVO_HOLD_TIME_MS (motor stays stopped, not spinning)
            HAL_Delay(SERVO_HOLD_TIME_MS);

            // Drive motor "close" direction for SERVO_MOVE_TIME_MS, then stop
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_REVERSE);
            HAL_Delay(SERVO_MOVE_TIME_MS);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_STOP);
            
            last_activity_time = HAL_GetTick(); // Reset inactivity timer after full scan & open cycle
        }
    }
    
    // 4. Trigger software reset if idle for 15 seconds
    if ((HAL_GetTick() - last_activity_time) >= TIMEOUT_THRESHOLD) {
        NVIC_SystemReset(); 
    }
    
    HAL_Delay(100); 
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  // Enable TIM2 Clock
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  // Prescaler = 72-1 -> Timer clock is 1MHz
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  // Period = 20000-1 -> 20ms period (50Hz) for standard servo
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure PA1 as Alternate Function Push-Pull for TIM2_CH2
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level for PC13 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); 

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_SET);
  
  // Set Relay, Buzzer, and LED_ALERT_Pin to boot LOW (RESET) to keep them OFF
  HAL_GPIO_WritePin(GPIOB, RELAY_CTRL_Pin|BUZZER_Pin|LED_ALERT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 (Built-in LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_INT_Pin */
  GPIO_InitStruct.Pin = TOF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOF_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RC522_RST_Pin */
  GPIO_InitStruct.Pin = RC522_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RC522_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RC522_CS_Pin */
  GPIO_InitStruct.Pin = RC522_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RC522_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VIB_SENSOR_Pin DOOR_SWITCH_Pin */
  GPIO_InitStruct.Pin = VIB_SENSOR_Pin|DOOR_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_CTRL_Pin BUZZER_Pin LED_ALERT_Pin */
  GPIO_InitStruct.Pin = RELAY_CTRL_Pin|BUZZER_Pin|LED_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* USER CODE BEGIN 4 */

void ESP8266_Send_User_Post(char* scanned_uid) {
    char atCommand[256];
    char httpRequest[512];
    char json_data[128];
    
    const char *server_ip = "192.168.0.232";
    
    sprintf(json_data, "{\"uid\": \"%s\", \"name\": \"Unknown\"}", scanned_uid);
    int json_len = strlen(json_data);

    sprintf(httpRequest,
            "POST /api/users HTTP/1.1\r\n"
            "Host: %s:3000\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n\r\n"
            "%s", server_ip, json_len, json_data);

    int http_len = strlen(httpRequest);

    sprintf(atCommand, "AT+CIPSTART=\"TCP\",\"%s\",3000\r\n", server_ip);
    HAL_UART_Transmit(&huart1, (uint8_t*)atCommand, strlen(atCommand), 1000);
    HAL_Delay(2000); 

    sprintf(atCommand, "AT+CIPSEND=%d\r\n", http_len);
    HAL_UART_Transmit(&huart1, (uint8_t*)atCommand, strlen(atCommand), 1000);
    HAL_Delay(500); 

    HAL_UART_Transmit(&huart1, (uint8_t*)httpRequest, http_len, 2000);
    HAL_Delay(1000); 
}

void ESP8266_Send_Log_Post(char* scanned_uid, const char* status) {
    char atCommand[256];
    char httpRequest[512];
    char json_data[128];
    
    const char *server_ip = "192.168.0.232"; 
    
    sprintf(json_data, "{\"uid\": \"%s\", \"status\": \"%s\"}", scanned_uid, status);
    int json_len = strlen(json_data);

    sprintf(httpRequest,
            "POST /api/logs HTTP/1.1\r\n"
            "Host: %s:3000\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n\r\n"
            "%s", server_ip, json_len, json_data);

    int http_len = strlen(httpRequest);

    sprintf(atCommand, "AT+CIPSTART=\"TCP\",\"%s\",3000\r\n", server_ip);
    HAL_UART_Transmit(&huart1, (uint8_t*)atCommand, strlen(atCommand), 1000);
    HAL_Delay(2000); 

    sprintf(atCommand, "AT+CIPSEND=%d\r\n", http_len);
    HAL_UART_Transmit(&huart1, (uint8_t*)atCommand, strlen(atCommand), 1000);
    HAL_Delay(500); 

    HAL_UART_Transmit(&huart1, (uint8_t*)httpRequest, http_len, 2000);
    HAL_Delay(1000); 
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */