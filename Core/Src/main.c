/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Leader/Follower
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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define NUM_MOTORS 6
#define NO_LEADER 255

// |Motor IDs|
uint8_t motor_ids[NUM_MOTORS] = {1, 2, 3, 4, 5, 6};

// |Feedback from motors|
float pos[NUM_MOTORS] = {0}; // motor position in radians
float vel[NUM_MOTORS] = {0}; // motor velocity in rad/s

// |Follow Direction|
// Direction for each motor: +1 = same, -1 = inverted
int8_t follow_direction[NUM_MOTORS] = {1, 1, 1, 1, 1, 1};

// |Leader/Follower Mapping|
// If leader_for_motor[i] = j → motor i follows motor j
// If leader_for_motor[i] = NO_LEADER → motor i is a leader
uint8_t leader_for_motor[NUM_MOTORS];

// |Offset|
float follow_offset[NUM_MOTORS];

// |PD Gains for Followers|
// Leader should be set to passive
//float Kp_arr[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 12.0f, 5.0f, 5.0f};
//float Kd_arr[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 1.0f, 0.5f, 0.5f};
float Kp_arr[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 48.0f, 24.0f, 24.0f};
float Kd_arr[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 2.0f, 2.0f, 2.0f};

// |Smoothed Command Position|
float cmd_pos_slow[NUM_MOTORS];

// |CAN Communication|
CAN_TxHeaderTypeDef txHeader; // CAN transmit header
CAN_RxHeaderTypeDef rxHeader; // CAN receive header

uint8_t txData[8]; // Data being sent to motor
uint8_t rxData[8]; // Data received from motor

uint32_t txMailbox; // Mailbox for CAN transmission

// |Feedback Validity|
uint8_t pos_valid[NUM_MOTORS] = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// |Convert CAN ID|
int motor_index_from_id(uint8_t id)
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (motor_ids[i] == id) return i;
    }
    return -1;
}

// |Assign Follower to Leader|
// Example: set_follower(2,1) for motor 2 follows motor 1
void set_follower(uint8_t follower_id, uint8_t leader_id, int8_t direction)
{
    int f = motor_index_from_id(follower_id);
    int l = motor_index_from_id(leader_id);

    if (f < 0 || l < 0 || f == l) return;

    leader_for_motor[f] = l;
    follow_direction[f] = direction;

    follow_offset[f] = pos[f] - direction * pos[l];

    cmd_pos_slow[f] = pos[f];
}

// |Pack Command|
// Converts float commands into 8-byte CAN frame (MIT mode)
void pack_cmd(float p, float v, float kp, float kd, float t)
{
    // Scale physical values to integer ranges
    uint16_t p_int = (uint16_t)((p + 12.5f) * 65535.0f / 25.0f);
    uint16_t v_int = (uint16_t)((v + 30.0f) * 4095.0f / 60.0f);
    uint16_t kp_int = (uint16_t)(kp * 4095.0f / 500.0f);
    uint16_t kd_int = (uint16_t)(kd * 4095.0f / 5.0f);
    uint16_t t_int = (uint16_t)((t + 18.0f) * 4095.0f / 36.0f);

    // Pack into CAN frame (bit-level format)
    txData[0] = p_int >> 8;
    txData[1] = p_int & 0xFF;
    txData[2] = v_int >> 4;
    txData[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    txData[4] = kp_int & 0xFF;
    txData[5] = kd_int >> 4;
    txData[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    txData[7] = t_int & 0xFF;
}


// |Unpack Feedback|
// Converts CAN reply into position and velocity
void unpack_reply(float *pos_out, float *vel_out)
{
    // [ID][POS_H][POS_L][VEL_H][VEL_L/KP_H]
    uint16_t p_int = ((uint16_t)rxData[1] << 8) | rxData[2];
    uint16_t v_int = ((uint16_t)rxData[3] << 4) | (rxData[4] >> 4);

    *pos_out = ((float)p_int) * 25.0f / 65535.0f - 12.5f;
    *vel_out = ((float)v_int) * 60.0f / 4095.0f - 30.0f;
}


// |Send Command|
// Sends packed command to a specific motor ID
void send_cmd(uint8_t id)
{
    txHeader.StdId = id;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}


// |Enable Motor|
// Sends enable command (MIT mode)
void enable_motor(uint8_t id)
{
    // Fill with 0xFF
    for (int i = 0; i < 8; i++)
        txData[i] = 0xFF;

    txData[7] = 0xFC; // Enable command

    txHeader.StdId = id;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

void disable_motor(uint8_t id)
{
    for (int i = 0; i < 8; i++)
        txData[i] = 0xFF;

    txData[7] = 0xFD;  // MIT disable

    txHeader.StdId = id;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start CAN peripheral
  HAL_CAN_Start(&hcan1);

  // Enable all motors
  // Controller enables motors

  // Allow time for initial feedback
  HAL_Delay(100);

  // |Initialize Mapping|
  for (int i = 0; i < NUM_MOTORS; i++)
  {
      leader_for_motor[i] = NO_LEADER;
      follow_offset[i] = 0.0f;
      cmd_pos_slow[i] = 0.0f;
  }

  // |Enable Motors|
  for (int k = 0; k < 5; k++)
  {
	  for (int i = 0; i < NUM_MOTORS; i++)
	  {
	      while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
			  enable_motor(motor_ids[i]);
		  HAL_Delay(5);
	  }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // |CAN Receiving|
      while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
      {
          HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData);

          // Match incoming CAN ID to one of the motors
          for (int i = 0; i < NUM_MOTORS; i++)
          {
              if (rxHeader.StdId == motor_ids[i])
              {
                  // Update position and velocity from CAN reply
                  unpack_reply(&pos[i], &vel[i]);

                  // Mark that this motor has reported at least once
                  pos_valid[i] = 1;
                  break;
              }
          }
      }

      // Check for Valid Data
      uint8_t all_valid = 1;

      for (int i = 0; i < NUM_MOTORS; i++)
      {
          if (!pos_valid[i])
          {
              all_valid = 0;
              break;
          }
      }

      // Data Output (Run logger.py)
      static uint32_t last_print = 0;
      if (HAL_GetTick() - last_print > 10)
      {
          printf("%lu", HAL_GetTick());

          for (int i = 0; i < NUM_MOTORS; i++)
          {
              printf(",%d", (int)(pos[i] * 1000));
          }

          printf("\r\n");
          last_print = HAL_GetTick();
      }

      // |Set Follower Mapping|
      static uint8_t mapping_set = 0;

      if (!mapping_set && all_valid)
      {
          HAL_Delay(500);  // let motors settle

          set_follower(6, 1, -1);
          set_follower(4, 2, -1);
          set_follower(5, 3, -1);

          mapping_set = 1;
      }

      // |Control Loop|
      static uint32_t last_time = 0;
      uint32_t now = HAL_GetTick();

      if (now - last_time >= 2)
      {
          last_time += 2;

          for (int i = 0; i < NUM_MOTORS; i++)
          {
              float kp = Kp_arr[i];
              float kd = Kd_arr[i];

              if (leader_for_motor[i] != NO_LEADER)
              {
                  // Follower
                  int leader = leader_for_motor[i];

                  // Target = leader position (with direction + offset)
                  float target = follow_direction[i] * pos[leader] + follow_offset[i];

                  // Light smoothing to prevent twitching/aggressive response
                  float alpha = 0.2f;
                  //cmd_pos_slow[i] = alpha * target + (1.0f - alpha) * cmd_pos_slow[i];
                  cmd_pos_slow[i] = target;

                  // Send PD position command
                  pack_cmd(cmd_pos_slow[i], 0.0f, kp, kd, 0.0f);
              }
              else
              {
                  // Leader (Passive)
                  pack_cmd(pos[i], 0.0f, 0.0f, 0.0f, 0.0f);
              }

              // Send Command
              while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
              {
                  // Wait for free mailbox
              }
              send_cmd(motor_ids[i]);
          }
      }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
	// Configured for 1Mbps CAN
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef filter;

  filter.FilterActivation = ENABLE;
  filter.FilterBank = 0;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow  = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow  = 0x0000;

  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  HAL_CAN_ConfigFilter(&hcan1, &filter);
  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
