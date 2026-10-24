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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t sequence = 0;
uint16_t lower_bound_pulse_width = 1000;

uint32_t commands_sent = 0;
uint32_t ack_first_try = 0;
uint32_t commands_retried = 0;
uint32_t commands_failed = 0;

uint32_t rtt_total_us = 0;
uint32_t rtt_min_us = 0xFFFFFFFF;
uint32_t rtt_max_us = 0;
uint32_t rtt_samples = 0;

uint32_t experiment_start_ms = 0;

volatile uint16_t latest_measured_pulse_us = 0;
volatile uint32_t telemetry_frames_received = 0;
volatile uint32_t telemetry_gaps = 0;

uint8_t last_telemetry_count = 0;
uint8_t telemetry_count_initialized = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t CRC16_MODBUS(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    return crc;
}


/* ---------- Frame structure ---------- */

typedef struct
{
    uint8_t type;
    uint8_t length;
    uint8_t payload[4];
} UART_Frame;


/* ---------- Send SET_PULSE command ---------- */

void SendPulseCommand(uint16_t pulse_us, uint8_t seq)
{
    uint8_t frame[8];

    frame[0] = 0xA5;
    frame[1] = 0x04;
    frame[2] = 0x01;
    frame[3] = seq;
    frame[4] = pulse_us & 0xFF;
    frame[5] = (pulse_us >> 8) & 0xFF;

    uint16_t crc = CRC16_MODBUS(&frame[1], 5);

    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    HAL_UART_Transmit(&huart1, frame, sizeof(frame), 100);
}


/* ---------- Receive and validate one frame ---------- */

uint8_t ReceiveFrame(UART_Frame *frame, uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < timeout_ms)
    {
        uint8_t start;
        uint8_t length;
        uint8_t payload[4];
        uint8_t crc_bytes[2];

        uint32_t elapsed = HAL_GetTick() - start_ms;
        uint32_t remaining = timeout_ms - elapsed;

        if (HAL_UART_Receive(&huart1, &start, 1, remaining) != HAL_OK)
            return 0;

        if (start != 0xA5)
            continue;

        elapsed = HAL_GetTick() - start_ms;
        if (elapsed >= timeout_ms)
            return 0;

        remaining = timeout_ms - elapsed;

        if (HAL_UART_Receive(&huart1, &length, 1, remaining) != HAL_OK)
            return 0;

        // ACK payload length = 3; telemetry payload length = 4
        if (length != 3 && length != 4)
            continue;

        elapsed = HAL_GetTick() - start_ms;
        if (elapsed >= timeout_ms)
            return 0;

        remaining = timeout_ms - elapsed;

        if (HAL_UART_Receive(&huart1, payload, length, remaining) != HAL_OK)
            return 0;

        elapsed = HAL_GetTick() - start_ms;
        if (elapsed >= timeout_ms)
            return 0;

        remaining = timeout_ms - elapsed;

        if (HAL_UART_Receive(&huart1, crc_bytes, 2, remaining) != HAL_OK)
            return 0;

        // CRC covers LENGTH + PAYLOAD
        uint8_t crc_data[5];
        crc_data[0] = length;

        for (uint8_t i = 0; i < length; i++)
            crc_data[i + 1] = payload[i];

        uint16_t calculated_crc =
            CRC16_MODBUS(crc_data, length + 1);

        uint16_t received_crc =
            (uint16_t)crc_bytes[0] |
            ((uint16_t)crc_bytes[1] << 8);

        if (received_crc != calculated_crc)
            continue;

        // Store the validated frame
        frame->length = length;
        frame->type = payload[0];

        for (uint8_t i = 0; i < length; i++)
            frame->payload[i] = payload[i];

        return 1;
    }

    return 0;
}


/* ---------- Process ACK or telemetry ---------- */

// Returns:
// 0 = telemetry, unrelated frame, or invalid ACK
// 1 = matching ACK accepted
// 2 = matching ACK rejected

uint8_t ProcessFrame(const UART_Frame *frame, uint8_t expected_seq)
{
    // ACK: A5 03 81 SEQ STATUS CRC_LO CRC_HI
    if (frame->type == 0x81 && frame->length == 3)
    {
        uint8_t seq = frame->payload[1];
        uint8_t status = frame->payload[2];

        if (seq != expected_seq)
            return 0;

        if (status == 0x00)
            return 1;

        if (status == 0x01)
            return 2;

        return 0;
    }

    // Telemetry: A5 04 82 COUNT PULSE_LO PULSE_HI CRC_LO CRC_HI
    if (frame->type == 0x82 && frame->length == 4)
    {
        uint8_t count = frame->payload[1];

        latest_measured_pulse_us =
            (uint16_t)frame->payload[2] |
            ((uint16_t)frame->payload[3] << 8);

        telemetry_frames_received++;

        if (telemetry_count_initialized &&
            count != (uint8_t)(last_telemetry_count + 1))
        {
            telemetry_gaps++;
        }

        last_telemetry_count = count;
        telemetry_count_initialized = 1;

        return 0;
    }

    return 0;
}


/* ---------- Send command and wait for its ACK ---------- */

void SendCommandWithAck(uint16_t pulse_us)
{
    uint8_t seq = sequence++;
    uint8_t result = 0;
    uint8_t retries_used = 0;
    uint32_t successful_rtt_us = 0;

    commands_sent++;

    for (uint8_t attempt = 0; attempt <= 3; attempt++)
    {
        uint32_t start_us = __HAL_TIM_GET_COUNTER(&htim2);

        SendPulseCommand(pulse_us, seq);

        uint32_t wait_start = HAL_GetTick();
        result = 0;

        while ((HAL_GetTick() - wait_start) < 20)
        {
            uint32_t elapsed = HAL_GetTick() - wait_start;
            uint32_t remaining = 20 - elapsed;

            UART_Frame frame;

            if (!ReceiveFrame(&frame, remaining))
                break;

            result = ProcessFrame(&frame, seq);

            if (result == 1 || result == 2)
                break;
        }

        uint32_t end_us = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t rtt_us = end_us - start_us;

        if (result == 1)
        {
            successful_rtt_us = rtt_us;
            retries_used = attempt;
            break;
        }

        if (result == 2)
        {
            printf("SEQ=%u rejected: invalid pulse\r\n", seq);
            return;
        }

        if (attempt < 3)
        {
            printf("SEQ=%u retry %u\r\n", seq, attempt + 1);
        }
    }

    if (result != 1)
    {
        commands_failed++;
        printf("SEQ=%u FAILED after 3 retries\r\n", seq);
        return;
    }

    if (retries_used == 0)
        ack_first_try++;
    else
        commands_retried++;

    rtt_total_us += successful_rtt_us;
    rtt_samples++;

    if (successful_rtt_us < rtt_min_us)
        rtt_min_us = successful_rtt_us;

    if (successful_rtt_us > rtt_max_us)
        rtt_max_us = successful_rtt_us;

    uint32_t elapsed_ms = HAL_GetTick() - experiment_start_ms;

    printf("%lu.%03lu s  seq=%3u  SET %4u us  ACK ok  RTT %lu us\r\n",
           (unsigned long)(elapsed_ms / 1000),
           (unsigned long)(elapsed_ms % 1000),
           seq,
           pulse_us,
           (unsigned long)successful_rtt_us);

    printf("Commanded: %u us | Measured: %u us | Error: %d us\r\n",
           pulse_us,
           latest_measured_pulse_us,
           (int)latest_measured_pulse_us - (int)pulse_us);
}

/* USER CODE END 0 */



//uint16_t CRC16_MODBUS(uint8_t *data, uint16_t length)
//{
//    uint16_t crc = 0xFFFF;
//
//    for (uint16_t i = 0; i < length; i++)
//    {
//        crc ^= data[i];
//
//        for (uint8_t bit = 0; bit < 8; bit++)
//        {
//            if (crc & 1)
//                crc = (crc >> 1) ^ 0xA001;
//            else
//                crc >>= 1;
//        }
//    }
//
//    return crc;
//}
//
//void SendPulseCommand(uint16_t pulse_us, uint8_t seq)
//{
//    uint8_t frame[8];
//
//    frame[0] = 0xA5;              // Start byte
//    frame[1] = 0x04;              // Payload length
//    frame[2] = 0x01;              // SET_PULSE
//    frame[3] = seq;               // Sequence number
//    frame[4] = pulse_us & 0xFF;   // Pulse low byte
//    frame[5] = pulse_us >> 8;      // Pulse high byte
//
//    uint16_t crc = CRC16_MODBUS(&frame[1], 5);
//
//    frame[6] = crc & 0xFF;         // CRC low byte
//    frame[7] = crc >> 8;           // CRC high byte
//
//    HAL_UART_Transmit(&huart1, frame, 8, 100);
//}
//
//
//uint8_t ReceiveAck(uint8_t expected_seq, uint32_t timeout_ms)
//{
//    uint32_t start_ms = HAL_GetTick();
//
//    while ((HAL_GetTick() - start_ms) < timeout_ms)
//    {
//        uint8_t start;
//        uint8_t length;
//        uint8_t payload[8];
//        uint8_t crc_bytes[2];
//
//        uint32_t elapsed = HAL_GetTick() - start_ms;
//        uint32_t remaining = timeout_ms - elapsed;
//
//        if (HAL_UART_Receive(&huart1, &start, 1, remaining) != HAL_OK)
//            return 0;
//
//        // Ignore bytes until the start marker
//        if (start != 0xA5)
//            continue;
//
//        remaining = timeout_ms - (HAL_GetTick() - start_ms);
//        if (remaining == 0 ||
//            HAL_UART_Receive(&huart1, &length, 1, remaining) != HAL_OK)
//            return 0;
//
//        // Length includes message type and other payload fields
//        if (length < 1 || length > sizeof(payload))
//            continue;
//
//        remaining = timeout_ms - (HAL_GetTick() - start_ms);
//        if (remaining == 0 ||
//            HAL_UART_Receive(&huart1, payload, length, remaining) != HAL_OK)
//            return 0;
//
//        remaining = timeout_ms - (HAL_GetTick() - start_ms);
//        if (remaining == 0 ||
//            HAL_UART_Receive(&huart1, crc_bytes, 2, remaining) != HAL_OK)
//            return 0;
//
//        // CRC covers length followed by the entire payload
//        uint8_t crc_data[9];
//        crc_data[0] = length;
//
//        for (uint8_t i = 0; i < length; i++)
//            crc_data[i + 1] = payload[i];
//
//        uint16_t calculated_crc =
//            CRC16_MODBUS(crc_data, length + 1);
//
//        uint16_t received_crc =
//            (uint16_t)crc_bytes[0] |
//            ((uint16_t)crc_bytes[1] << 8);
//
//        if (received_crc != calculated_crc)
//            continue;
//
//        // Telemetry frame: A5 04 82 COUNT PULSE_LO PULSE_HI CRC_LO CRC_HI
//        if (payload[0] == 0x82 && length == 4)
//        {
//            uint8_t count = payload[1];
//
//            latest_measured_pulse_us =
//                (uint16_t)payload[2] |
//                ((uint16_t)payload[3] << 8);
//
//            telemetry_frames_received++;
//
//            if (telemetry_count_initialized &&
//                count != (uint8_t)(last_telemetry_count + 1))
//            {
//                telemetry_gaps++;
//            }
//
//            last_telemetry_count = count;
//            telemetry_count_initialized = 1;
//
//            continue;  // Keep waiting for our ACK
//        }
//
//        // ACK frame: A5 03 81 SEQ STATUS CRC_LO CRC_HI
//        if (payload[0] == 0x81 && length == 3)
//        {
//            if (payload[1] != expected_seq)
//                continue;
//
//            if (payload[2] == 0x00)
//                return 1;  // ACK accepted
//
//            if (payload[2] == 0x01)
//                return 2;  // ACK rejected
//
//            continue;
//        }
//    }
//
//    return 0; // Timeout
//}
//
//
//void SendCommandWithAck(uint16_t pulse_us)
//{
//    uint8_t seq = sequence++;
//    uint8_t result = 0;
//    uint32_t successful_rtt_us = 0;
//    uint8_t retries_used = 0;
//
//    commands_sent++;
//
//    for (uint8_t attempt = 0; attempt <= 3; attempt++)
//    {
//        uint32_t start_us = __HAL_TIM_GET_COUNTER(&htim2);
//
//        SendPulseCommand(pulse_us, seq);
//
//        result = ReceiveAck(seq, 20);
//
//        uint32_t end_us = __HAL_TIM_GET_COUNTER(&htim2);
//        uint32_t rtt_us = end_us - start_us;
//
//        if (result == 1)
//        {
//            successful_rtt_us = rtt_us;
//            retries_used = attempt;
//            break;
//        }
//
//        if (result == 2)
//        {
//            // Valid ACK, but Board A rejected the pulse
//            printf("SEQ=%u rejected: invalid pulse\r\n", seq);
//            return;
//        }
//
//        if (attempt < 3)
//        {
//            printf("SEQ=%u retry %u\r\n",
//                   seq, attempt + 1);
//        }
//    }
//
//    if (result != 1)
//    {
//        commands_failed++;
//        printf("SEQ=%u FAILED after 3 retries\r\n", seq);
//        return;
//    }
//
//    if (retries_used == 0)
//        ack_first_try++;
//    else
//        commands_retried++;
//
//    rtt_total_us += successful_rtt_us;
//    rtt_samples++;
//
//    if (successful_rtt_us < rtt_min_us)
//        rtt_min_us = successful_rtt_us;
//
//    if (successful_rtt_us > rtt_max_us)
//        rtt_max_us = successful_rtt_us;
//
//    uint32_t elapsed_ms = HAL_GetTick() - experiment_start_ms;
//
//    printf("%lu.%03lu s  seq=%3u  SET %4u us  ACK ok  rtt %lu us\r\n",
//           (unsigned long)(elapsed_ms / 1000),
//           (unsigned long)(elapsed_ms % 1000),
//           seq,
//           pulse_us,
//           (unsigned long)successful_rtt_us);
//
//    printf("Telemetry frames: %lu\r\n",
//           (unsigned long)telemetry_frames_received);
//
//    printf("Telemetry gaps: %lu\r\n",
//           (unsigned long)telemetry_gaps);
//}
//
//uint8_t ProcessFrame(const UART_Frame *frame, uint8_t expected_seq)
//{
//    // ACK: A5 03 81 SEQ STATUS CRC_LO CRC_HI
//    if (frame->type == 0x81 && frame->length == 3)
//    {
//        uint8_t seq = frame->payload[1];
//        uint8_t status = frame->payload[2];
//
//        if (seq != expected_seq)
//            return 0;
//
//        if (status == 0x00)
//            return 1;
//
//        if (status == 0x01)
//            return 2;
//
//        return 0;
//    }
//
//    // Telemetry: A5 04 82 COUNT PULSE_LO PULSE_HI CRC_LO CRC_HI
//    if (frame->type == 0x82 && frame->length == 4)
//    {
//        uint8_t count = frame->payload[1];
//
//        latest_measured_pulse_us =
//            (uint16_t)frame->payload[2] |
//            ((uint16_t)frame->payload[3] << 8);
//
//        telemetry_frames_received++;
//
//        if (telemetry_count_initialized &&
//            count != (uint8_t)(last_telemetry_count + 1))
//        {
//            telemetry_gaps++;
//        }
//
//        last_telemetry_count = count;
//        telemetry_count_initialized = 1;
//
//        return 0;
//    }
//
//    // Unknown message type
//    return 0;
//}
//
//
//
//uint8_t ReceiveFrame(UART_Frame *frame, uint32_t timeout_ms)
//{
//    uint32_t start_ms = HAL_GetTick();
//
//    while ((HAL_GetTick() - start_ms) < timeout_ms)
//    {
//        uint8_t start;
//        uint8_t length;
//        uint8_t payload[4];
//        uint8_t crc_bytes[2];
//
//        uint32_t elapsed = HAL_GetTick() - start_ms;
//        uint32_t remaining = timeout_ms - elapsed;
//
//        if (HAL_UART_Receive(&huart1, &start, 1, remaining) != HAL_OK)
//            return 0;
//
//        // Find the frame start byte
//        if (start != 0xA5)
//            continue;
//
//        elapsed = HAL_GetTick() - start_ms;
//        if (elapsed >= timeout_ms)
//            return 0;
//
//        remaining = timeout_ms - elapsed;
//
//        if (HAL_UART_Receive(&huart1, &length, 1, remaining) != HAL_OK)
//            return 0;
//
//        // Our protocol supports ACK (length 3) and telemetry (length 4)
//        if (length != 3 && length != 4)
//            continue;
//
//        elapsed = HAL_GetTick() - start_ms;
//        if (elapsed >= timeout_ms)
//            return 0;
//
//        remaining = timeout_ms - elapsed;
//
//        if (HAL_UART_Receive(&huart1, payload, length, remaining) != HAL_OK)
//            return 0;
//
//        elapsed = HAL_GetTick() - start_ms;
//        if (elapsed >= timeout_ms)
//            return 0;
//
//        remaining = timeout_ms - elapsed;
//
//        if (HAL_UART_Receive(&huart1, crc_bytes, 2, remaining) != HAL_OK)
//            return 0;
//
//        // CRC covers LENGTH + PAYLOAD
//        uint8_t crc_data[5];
//        crc_data[0] = length;
//
//        for (uint8_t i = 0; i < length; i++)
//            crc_data[i + 1] = payload[i];
//
//        uint16_t calculated_crc =
//            CRC16_MODBUS(crc_data, length + 1);
//
//        uint16_t received_crc =
//            (uint16_t)crc_bytes[0] |
//            ((uint16_t)crc_bytes[1] << 8);
//
//        if (received_crc != calculated_crc)
//            continue;
//
//        // Copy the validated frame to the output structure
//        frame->length = length;
//        frame->type = payload[0];
//
//        for (uint8_t i = 0; i < length; i++)
//            frame->payload[i] = payload[i];
//
//        return 1;
//    }
//
//    return 0;
//}
//
//
//

void Run1000Commands(void)
{
    while (commands_sent < 1000)
    {
        SendCommandWithAck(lower_bound_pulse_width + commands_sent);
        HAL_Delay(100);
    }

    printf("\r\n===== 1000 COMMAND SUMMARY =====\r\n");

    printf("Commands sent: %lu\r\n",
           (unsigned long)commands_sent);

    printf("ACKed first try: %lu\r\n",
           (unsigned long)ack_first_try);

    printf("Needed retries: %lu\r\n",
           (unsigned long)commands_retried);

    printf("Failed: %lu\r\n",
           (unsigned long)commands_failed);

    if (rtt_samples > 0)
    {
        printf("Mean RTT: %lu us\r\n",
               (unsigned long)(rtt_total_us / rtt_samples));

        printf("Worst successful RTT: %lu us\r\n",
               (unsigned long)rtt_max_us);

        printf("Best successful RTT: %lu us\r\n",
               (unsigned long)rtt_min_us);
    }

    printf("\r\n===== TELEMETRY SUMMARY =====\r\n");

    printf("Telemetry frames received: %lu\r\n",
           (unsigned long)telemetry_frames_received);

    printf("Telemetry sequence gaps: %lu\r\n",
           (unsigned long)telemetry_gaps);

    printf("Latest measured pulse: %u us\r\n",
           latest_measured_pulse_us);

    printf("================================\r\n");

    while (1)
    {
        HAL_Delay(1000);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  experiment_start_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      Run1000Commands();
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  htim2.Init.Prescaler = 83;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
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
