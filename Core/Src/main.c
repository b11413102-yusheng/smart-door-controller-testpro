/* USER CODE BEGIN Header */
/**
  * ******************************************************************************
  * @file           : main.c
  * @brief          : Smart Rolling Shutter Control System (STM32G474RE)
  * ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_STANDBY = 0,
    STATE_UP,
    STATE_DOWN,
    STATE_EMERGENCY,
    STATE_PAUSE
} SystemState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY_MS   10
#define SHUTTER_TRAVEL_MS   5000
#define PAUSE_DISPLAY_MS    3000
#define EMERGENCY_BLINK_MS  250
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* 狀態機與系統變數 */
volatile SystemState_t current_state = STATE_STANDBY;
volatile SystemState_t previous_state = STATE_STANDBY;

/* 系統日誌計數器 */
uint32_t total_ops_count = 0;
uint32_t up_count = 0;
uint32_t down_count = 0;
uint32_t stop_count = 0;
uint32_t emergency_count = 0;

/* 中斷與旗標變數 */
volatile uint8_t rx_char = 0;
volatile uint8_t emergency_flag = 0;
volatile uint8_t cmd_up_flag = 0;
volatile uint8_t cmd_down_flag = 0;
volatile uint8_t cmd_stop_flag = 0;
volatile uint8_t cmd_emergency_flag = 0;

/* 非阻塞計時基準點 */
uint32_t state_start_tick = 0;
uint32_t last_debounce_tick = 0;
uint32_t last_blink_tick = 0;
uint8_t emergency_led_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Log_System_Status(const char* cmd_name);
void Process_Commands(void);
void Button_Poll_Handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  PLC 聯網日誌輸出函式
  */
void Log_System_Status(const char* cmd_name) {
    char log_buf[128];
    uint32_t uptime_sec = HAL_GetTick() / 1000;

    int len = snprintf(log_buf, sizeof(log_buf),
        "[PLC LOG] Uptime: %lu s | Cmd: %s | Total: %lu | Up: %lu | Down: %lu | Stop: %lu | Emergency: %lu\r\n",
        uptime_sec, cmd_name, total_ops_count, up_count, down_count, stop_count, emergency_count);

    HAL_UART_Transmit(&huart2, (uint8_t*)log_buf, len, HAL_MAX_DELAY);
}

/**
  * @brief  UART 接收中斷回呼函式
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (rx_char == 'u' || rx_char == 'U') {
            cmd_up_flag = 1;
        } else if (rx_char == 'd' || rx_char == 'D') {
            cmd_down_flag = 1;
        } else if (rx_char == 's' || rx_char == 'S') {
            cmd_stop_flag = 1;
        } else if (rx_char == 'e' || rx_char == 'E' || rx_char == 'c' || rx_char == 'C') {
            cmd_emergency_flag = 1;
        }
        // 重新啟動接收中斷
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_char, 1);
    }
}

/**
  * @brief  外部中斷回呼函式 (處理緊急按鈕 PC9)
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == Btn_Emergency_Pin) {
        cmd_emergency_flag = 1;
    }
}

/**
  * @brief  按鈕 10ms 防彈跳輪詢處理
  */
void Button_Poll_Handler(void) {
    if ((HAL_GetTick() - last_debounce_tick) < DEBOUNCE_DELAY_MS) {
        return;
    }
    last_debounce_tick = HAL_GetTick();

    // PC6: Btn_Up (Active Low with Pull-up)
    if (HAL_GPIO_ReadPin(Btn_Up_GPIO_Port, Btn_Up_Pin) == GPIO_PIN_RESET) {
        if (current_state == STATE_STANDBY || current_state == STATE_PAUSE) {
            cmd_up_flag = 1;
        }
    }
    // PC7: Btn_Down (Active Low with Pull-up)
    if (HAL_GPIO_ReadPin(Btn_Down_GPIO_Port, Btn_Down_Pin) == GPIO_PIN_RESET) {
        if (current_state == STATE_STANDBY || current_state == STATE_PAUSE) {
            cmd_down_flag = 1;
        }
    }
    // PC8: Btn_Stop (Active Low with Pull-up)
    if (HAL_GPIO_ReadPin(Btn_Stop_GPIO_Port, Btn_Stop_Pin) == GPIO_PIN_RESET) {
        if (current_state == STATE_UP || current_state == STATE_DOWN) {
            cmd_stop_flag = 1;
        }
    }
}

/**
  * @brief  統一指令解析與狀態切換分派
  */
void Process_Commands(void) {
    // 優先處理緊急中斷/指令
    if (cmd_emergency_flag) {
        cmd_emergency_flag = 0;
        emergency_count++;
        total_ops_count++;

        if (current_state != STATE_EMERGENCY) {
            current_state = STATE_EMERGENCY;
            Log_System_Status("EMERGENCY_TRIGGER");
        } else {
            // 再次觸發解除互鎖回到待機
            current_state = STATE_STANDBY;
            Log_System_Status("EMERGENCY_RELEASE");
        }
        // 清除所有 PWM 與動作輸出
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
        return;
    }

    // 若處於緊急狀態，阻擋其他普通指令
    if (current_state == STATE_EMERGENCY) {
        cmd_up_flag = 0;
        cmd_down_flag = 0;
        cmd_stop_flag = 0;
        return;
    }

    // 處理上升指令
    if (cmd_up_flag) {
        cmd_up_flag = 0;
        if (current_state == STATE_STANDBY || current_state == STATE_PAUSE) {
            up_count++;
            total_ops_count++;
            current_state = STATE_UP;
            state_start_tick = HAL_GetTick();

            // 從暫停切換回上升時，強制關閉紅燈
            HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);

            // 啟動 TIM2 綠燈 PWM
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

            Log_System_Status("CMD_UP_START");
        }
    }

    // 處理下降指令
    if (cmd_down_flag) {
        cmd_down_flag = 0;
        if (current_state == STATE_STANDBY || current_state == STATE_PAUSE) {
            down_count++;
            total_ops_count++;
            current_state = STATE_DOWN;
            state_start_tick = HAL_GetTick();

            // 從暫停切換回下降時，強制關閉紅燈
            HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);

            // 啟動 TIM3 黃燈 PWM
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

            Log_System_Status("CMD_DOWN_START");
        }
    }

    // 處理停止指令
    if (cmd_stop_flag) {
        cmd_stop_flag = 0;
        if (current_state == STATE_UP || current_state == STATE_DOWN) {
            stop_count++;
            total_ops_count++;
            current_state = STATE_PAUSE;
            state_start_tick = HAL_GetTick();

            // 停止 PWM 並點亮紅燈
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);

            Log_System_Status("CMD_STOP");
        }
    }
}
/* USER CODE END P0 */

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

  /* Initialize all peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // 啟動 UART 接收中斷
  HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_char, 1);

  // 初始系統狀態提示
  char *welcome_msg = "\r\n=== NUCLEO-G474RE Smart Rolling Shutter System Initialized ===\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)welcome_msg, strlen(welcome_msg), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 1. 輪詢防彈跳按鈕
      Button_Poll_Handler();

      // 2. 處理指令與狀態轉換
      Process_Commands();

      // 3. 有限狀態機 (FSM) 運行邏輯
      switch (current_state)
      {
          case STATE_STANDBY:
              HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(Emergency_LED_GPIO_Port, Emergency_LED_Pin, GPIO_PIN_RESET);
              break;

          case STATE_UP:
          {
              uint32_t elapsed = HAL_GetTick() - state_start_tick;
              if (elapsed >= SHUTTER_TRAVEL_MS) {
                  current_state = STATE_PAUSE;
                  state_start_tick = HAL_GetTick();
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
                  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                  HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
                  stop_count++;
                  total_ops_count++;
                  Log_System_Status("UP_COMPLETE_AUTO_PAUSE");
              } else {
                  uint32_t duty = (elapsed * 99) / SHUTTER_TRAVEL_MS;
                  if (duty > 99) duty = 99;
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
              }
              break;
          }

          case STATE_DOWN:
          {
              uint32_t elapsed = HAL_GetTick() - state_start_tick;
              if (elapsed >= SHUTTER_TRAVEL_MS) {
                  current_state = STATE_PAUSE;
                  state_start_tick = HAL_GetTick();
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                  HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
                  stop_count++;
                  total_ops_count++;
                  Log_System_Status("DOWN_COMPLETE_AUTO_PAUSE");
              } else {
                  uint32_t duty = (elapsed * 99) / SHUTTER_TRAVEL_MS;
                  if (duty > 99) duty = 99;
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
              }
              break;
          }

          case STATE_PAUSE:
          {
              if ((HAL_GetTick() - state_start_tick) >= PAUSE_DISPLAY_MS) {
                  HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
                  current_state = STATE_STANDBY;
                  Log_System_Status("PAUSE_TIMEOUT_TO_STANDBY");
              }
              break;
          }

          case STATE_EMERGENCY:
          {
              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
              HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);

              if ((HAL_GetTick() - last_blink_tick) >= EMERGENCY_BLINK_MS) {
                  last_blink_tick = HAL_GetTick();
                  emergency_led_state ^= 1;
                  HAL_GPIO_WritePin(Emergency_LED_GPIO_Port, Emergency_LED_Pin,
                                    emergency_led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
              }
              break;
          }

          default:
              current_state = STATE_STANDBY;
              break;
      }

      HAL_Delay(1);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 170 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_SLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 170 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_SLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Emergency_LED_GPIO_Port, Emergency_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = Btn_Up_Pin | Btn_Down_Pin | Btn_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 (Emergency Button EXTI) */
  GPIO_InitStruct.Pin = Btn_Emergency_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Btn_Emergency_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 Red_LED */
  GPIO_InitStruct.Pin = Red_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Red_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 Emergency_LED */
  GPIO_InitStruct.Pin = Emergency_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Emergency_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init-> NVIC Priority */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
// 注意：HAL_TIM_MspPostInit 已交由 CubeMX 自動在 stm32g4xx_hal_msp.c 中生成，此處保持空白避免重複定義
/* USER CODE END 4 */

/**
  * @brief  This application is executed in the case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
