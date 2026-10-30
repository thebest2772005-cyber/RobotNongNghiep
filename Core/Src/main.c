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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  AXIS_NONE = 0,
  AXIS_U,
  AXIS_R
} ControlAxis_t;

/**
 * @brief Union ghép 4 byte từ UART thành biến float (chuẩn IEEE 754)
 */
typedef union {
  float    f_val;      /*!< Giá trị float (4 bytes) */
  uint8_t  bytes[4];   /*!< 4 bytes tương ứng (Little-Endian: bytes[0] là LSB) */
} FloatBytesUnion_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SBUS_FRAME_SIZE 25U
#define SBUS_RX_BUF_SIZE 64U
#define SBUS_STREAM_BUF_SIZE (SBUS_RX_BUF_SIZE + SBUS_FRAME_SIZE - 1U)
#define SBUS_CENTER 992.0f
#define SBUS_DEADBAND 150.0f
#define AUTO_MIN_SPEED 0.0f /* Tốc độ khởi động tối thiểu khi chạy thẳng (%) */
#define PID_MAX 15.0f
#define PID_MIN -15.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
volatile uint8_t SCALE = 1;
float delta_u, delta_r;
uint16_t pre_value_ch4 = 1000;
uint16_t pre_value_ch5 = 1000;
uint16_t pre_value_ch6 = 1000;
uint16_t pre_value_ch7 = 1000;
volatile int16_t pwm_test1 =
    0; /* Động cơ 1: Tốc độ từ -100 đến 100 (%) (PWM: PA0, DIR: PB1) */
volatile int16_t pwm_test2 =
    0; /* Động cơ 2: Tốc độ từ -100 đến 100 (%) (PWM: PA1, DIR: PB0) */
volatile int16_t pwm_test = 0; /* Biến test chung */

/* --- THÔNG SỐ ĐỘNG HỌC ROBOT (DIFFERENTIAL DRIVE KINEMATICS) --- */
float a = 1.0f;              /* Bán kính bánh xe (m) */
float d = 1.0f;              /* Khoảng cách từ bánh xe đến tâm robot (m) */
volatile float cmd_u = 0.0f; /* Vận tốc dài tịnh tiến (u) - tiến / lùi */
volatile float cmd_u_base = 0.0f; /* Vận tốc dài cơ sở chưa giảm tốc khi bẻ lái */
volatile float cmd_r = 0.0f; /* Vận tốc góc quay (r) - xoay trái / phải */

volatile float alpha1 =
    1.0f; /* Hệ số tỉ lệ tốc độ tổng duy nhất (0.0 -> 1.0), nhân trực tiếp vào cmd_u và cmd_r */
volatile float alpha2 =
    1.0f; /* Hệ số dự phòng (không sử dụng cho bánh xe nữa) */
float omega1 = 0.0f; /* Vận tốc góc bánh phải (Motor 1) */
float omega2 = 0.0f; /* Vận tốc góc bánh trái (Motor 2) */

/* --- BIẾN VÀ BỘ ĐỆM GIẢI MÃ SBUS (16 KÊNH) --- */
volatile uint8_t sbus_frame[SBUS_FRAME_SIZE];
volatile uint16_t
    sbus_channel[16]; /* 16 kênh RC: Min ~172, Tâm ~992, Max ~1811 */
volatile uint8_t sbus_frame_lost = 0;
volatile uint8_t sbus_failsafe = 0;
volatile uint8_t sbus_sync_ok = 0;

uint8_t sbus_dma_buffer[SBUS_RX_BUF_SIZE];
uint8_t sbus_stream_buffer[SBUS_STREAM_BUF_SIZE];
volatile uint16_t sbus_parser_count = 0;

/* --- BIẾN ĐIỀU KHIỂN BƠM (PUMP CONTROL) & TỰ ĐỘNG TIẾN (AUTO FORWARD) --- */
volatile uint8_t Pump_control = 0; /* 1: Bật bơm, 0: Tắt bơm */
volatile uint8_t Auto_Forward =0; /* 1: Tự động chạy thẳng, 0: Chạy theo cần gạt tay */
volatile uint8_t pre_Auto_Forward = 0;
volatile uint8_t ramp_done = 0; /* 1: Đã hoàn thành speed_ramp (cmd_u >= target_speed), 0: Đang chạy ramp */
volatile float auto_accel_time_ms = 3000.0f; /* <<< ĐIỀN THỜI GIAN GIA TỐC (ms) VÀO ĐÂY: Ví dụ 2000ms = 2 giây để đạt tốc độ */
volatile uint8_t auto_turn_active = 0;
volatile uint8_t auto_turn_armed = 1;
volatile uint32_t auto_turn_start_tick = 0;
volatile uint32_t auto_turn_end_tick = 0;
volatile uint32_t auto_turn_duration_ms = 150; /* Thời gian xoay tối đa (ms) khi gạt ngang (150ms) */
volatile uint8_t manual_turn_active = 0;
volatile uint8_t manual_turn_armed = 1;
volatile uint32_t manual_turn_start_tick = 0;
volatile uint32_t manual_turn_end_tick = 0;
volatile uint32_t turn_debounce_ms = 400; /* Thời gian nghỉ 100ms chống spam giữa các lần gạt xoay */
volatile uint8_t is_forward = 0; /* 1: Đang tiến (cmd_u_base > 0), 0: Đứng yên hoặc lùi */

/* --- BIẾN ĐIỀU KHIỂN ƯU TIÊN TRỤC (AXIS PRIORITY) --- */
volatile ControlAxis_t control_axis = AXIS_NONE;
volatile ControlAxis_t previous_control_axis = AXIS_NONE;
float pre_rc_u = 0.0f;
float pre_rc_r = 0.0f;

/* --- BIẾN NHẬN VÀ LƯU DỮ LIỆU TỪ UART1 QUA DMA IDLE (GÓI 5 BYTE) --- */
#define UART1_RX_BUF_SIZE 20U
uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];
FloatBytesUnion_t uart1_path_union;
volatile float error_path = 0.0f;          /*!< Lỗi lệch đường (float ghép từ 4 byte đầu qua union) */
volatile uint8_t is_path = 0;              /*!< Trạng thái đường: 1: Có đường, 0: Mất đường (byte thứ 5) */
volatile uint8_t qr_stop_flag = 0;         /*!< Cờ trạm dừng QR (>200x200): 1: Dừng robot, 0: Bình thường (byte thứ 5 từ UART1) */
volatile uint8_t uart1_data[5] = {0};      /*!< Mảng 5 byte lưu dữ liệu thô nhận từ UART1 */
volatile uint16_t uart1_rx_len = 0;        /*!< Chiều dài byte thực tế nhận được */
volatile uint8_t uart1_data_updated = 0;   /*!< Cờ báo vừa nhận được dữ liệu mới (1: có dữ liệu mới) */
volatile uint32_t uart1_rx_count = 0;      /*!< Đếm số lần nhận gói tin thành công từ UART1 */

/* --- THÔNG SỐ VÀ BIẾN PID BÁM ĐƯỜNG (PID_PATH) --- */
float target_path = 0.0f;           /*!< Mục tiêu vị trí lệch đường (0.0 là giữa tâm) */
volatile float err_path, pre_err_path;
float kp_path = 2.0f, kd_path = 1.0f, ki_path = 0.0f; /*!< Thông số PID Path */
volatile float up_path, ud_path, ui_path, uout_path;
volatile float ud_fil_path, ud_fil_pre_path;
float alpha_fil_path = 0.8f;
float deltaT_path = 0.005f;         /*!< Chu kỳ tính PID (5ms tương ứng vòng lặp HAL_Delay(5)) */
volatile uint8_t path_pid_active = 0; /*!< 1: Bật chế độ tự động bẻ lái theo PID Path (Núm 2 > 0.5), 0: Theo cần gạt */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Motor1_SetSpeed(int16_t pwm_val);
void Motor2_SetSpeed(int16_t pwm_val);
void Motor_SetSpeed(int16_t pwm_val);
float map2(float x, float in_min, float in_max, float out_min, float out_max);
void Kinematic_Cal(float u, float r);

static void SBUS_StartReceive(void);
static void SBUS_DecodeChannels(void);
static uint16_t SBUS_ParseBytes(const uint8_t *data, uint16_t size);
static void UART1_StartReceive(void);
float SBUS_Knob_To_Speed(uint16_t ch_val);
float SBUS_Knob_To_Alpha(uint16_t ch_val);
float Filter_Alpha(float current_alpha, uint16_t ch_val);
float Speed_Ramp(float current_speed, float target_speed, float time_ms);
float pid_path(float target);
void pid_reset_path(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  Khởi động nhận dữ liệu SBUS qua DMA (Receive To Idle)
 */
static void SBUS_StartReceive(void) {
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, sbus_dma_buffer,
                                   SBUS_RX_BUF_SIZE) == HAL_OK) {
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); /* Tắt ngắt Half-Transfer */
  }
}

/**
 * @brief  Khởi động nhận dữ liệu UART1 qua DMA Idle Line
 */
static void UART1_StartReceive(void) {
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buf,
                                   UART1_RX_BUF_SIZE) == HAL_OK) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT); /* Tắt ngắt Half-Transfer */
  }
}

/**
 * @brief  Giải nén 16 kênh 11-bit từ frame SBUS 25 bytes
 */
static void SBUS_DecodeChannels(void) {
  sbus_channel[0] =
      ((uint16_t)sbus_frame[1] | ((uint16_t)sbus_frame[2] << 8)) & 0x07FFU;
  sbus_channel[1] =
      ((uint16_t)sbus_frame[2] >> 3 | ((uint16_t)sbus_frame[3] << 5)) & 0x07FFU;
  sbus_channel[2] =
      ((uint16_t)sbus_frame[3] >> 6 | ((uint16_t)sbus_frame[4] << 2) |
       ((uint16_t)sbus_frame[5] << 10)) &
      0x07FFU;
  sbus_channel[3] =
      ((uint16_t)sbus_frame[5] >> 1 | ((uint16_t)sbus_frame[6] << 7)) & 0x07FFU;
  sbus_channel[4] =
      ((uint16_t)sbus_frame[6] >> 4 | ((uint16_t)sbus_frame[7] << 4)) & 0x07FFU;
  sbus_channel[5] =
      ((uint16_t)sbus_frame[7] >> 7 | ((uint16_t)sbus_frame[8] << 1) |
       ((uint16_t)sbus_frame[9] << 9)) &
      0x07FFU;
  sbus_channel[6] =
      ((uint16_t)sbus_frame[9] >> 2 | ((uint16_t)sbus_frame[10] << 6)) &
      0x07FFU;
  sbus_channel[7] =
      ((uint16_t)sbus_frame[10] >> 5 | ((uint16_t)sbus_frame[11] << 3)) &
      0x07FFU;
  sbus_channel[8] =
      ((uint16_t)sbus_frame[12] | ((uint16_t)sbus_frame[13] << 8)) & 0x07FFU;
  sbus_channel[9] =
      ((uint16_t)sbus_frame[13] >> 3 | ((uint16_t)sbus_frame[14] << 5)) &
      0x07FFU;
  sbus_channel[10] =
      ((uint16_t)sbus_frame[14] >> 6 | ((uint16_t)sbus_frame[15] << 2) |
       ((uint16_t)sbus_frame[16] << 10)) &
      0x07FFU;
  sbus_channel[11] =
      ((uint16_t)sbus_frame[16] >> 1 | ((uint16_t)sbus_frame[17] << 7)) &
      0x07FFU;
  sbus_channel[12] =
      ((uint16_t)sbus_frame[17] >> 4 | ((uint16_t)sbus_frame[18] << 4)) &
      0x07FFU;
  sbus_channel[13] =
      ((uint16_t)sbus_frame[18] >> 7 | ((uint16_t)sbus_frame[19] << 1) |
       ((uint16_t)sbus_frame[20] << 9)) &
      0x07FFU;
  sbus_channel[14] =
      ((uint16_t)sbus_frame[20] >> 2 | ((uint16_t)sbus_frame[21] << 6)) &
      0x07FFU;
  sbus_channel[15] =
      ((uint16_t)sbus_frame[21] >> 5 | ((uint16_t)sbus_frame[22] << 3)) &
      0x07FFU;

  sbus_frame_lost = ((sbus_frame[23] & 0x04U) != 0U);
  sbus_failsafe = ((sbus_frame[23] & 0x08U) != 0U);
}

/**
 * @brief  Tìm kiếm Header 0x0F và Footer 0x00 của frame SBUS
 */
static uint16_t SBUS_ParseBytes(const uint8_t *data, uint16_t size) {
  uint16_t scan_index = 0U;
  uint16_t valid_frames = 0U;

  if (size > SBUS_RX_BUF_SIZE)
    size = SBUS_RX_BUF_SIZE;
  for (uint16_t i = 0; i < size; i++) {
    sbus_stream_buffer[sbus_parser_count++] = data[i];
  }

  while ((sbus_parser_count - scan_index) >= SBUS_FRAME_SIZE) {
    if (sbus_stream_buffer[scan_index] != 0x0FU ||
        sbus_stream_buffer[scan_index + SBUS_FRAME_SIZE - 1U] != 0x00U) {
      scan_index++;
      continue;
    }
    for (uint16_t i = 0; i < SBUS_FRAME_SIZE; i++) {
      sbus_frame[i] = sbus_stream_buffer[scan_index + i];
    }
    SBUS_DecodeChannels();
    valid_frames++;
    scan_index += SBUS_FRAME_SIZE;
  }

  uint16_t remaining = sbus_parser_count - scan_index;
  for (uint16_t i = 0; i < remaining; i++) {
    sbus_stream_buffer[i] = sbus_stream_buffer[scan_index + i];
  }
  sbus_parser_count = remaining;
  return valid_frames;
}

/**
 * @brief  Callback ngắt nhận IDLE UART từ DMA (USART1 và USART2)
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART2) {
    if (SBUS_ParseBytes(sbus_dma_buffer, Size) > 0U) {
      sbus_sync_ok = 1U;
    }
    SBUS_StartReceive();
  }
  if (huart->Instance == USART1) {
    if (Size >= 5U) {
      /* Ghép 4 byte đầu (0..3) thành biến float error_path qua union */
      uart1_path_union.bytes[0] = uart1_rx_buf[0];
      uart1_path_union.bytes[1] = uart1_rx_buf[1];
      uart1_path_union.bytes[2] = uart1_rx_buf[2];
      uart1_path_union.bytes[3] = uart1_rx_buf[3];
      error_path = uart1_path_union.f_val;

      /* Byte thứ 5 (index 4) là cờ trạm dừng QR (> 200x200 pixels) từ tầng trên:
       * - 1: Kích hoạt trạm dừng (QR > 200x200 px) -> Robot dừng lại
       * - 0: Trạng thái chạy bình thường (bám đường hoặc bám tâm QR)
       */
      qr_stop_flag = uart1_rx_buf[4];
      is_path = (qr_stop_flag == 1) ? 0 : 1;
    }

    if (Size > 0U) {
      /* Sao chép 5 byte vào mảng dữ liệu thô uart1_data */
      uint8_t copy_len = (Size >= sizeof(uart1_data)) ? (uint8_t)sizeof(uart1_data) : (uint8_t)Size;
      for (uint8_t i = 0; i < copy_len; i++) {
        uart1_data[i] = uart1_rx_buf[i];
      }
      uart1_rx_len = Size;
      uart1_data_updated = 1;
      uart1_rx_count++;
    }
    /* Khởi động lại nhận DMA Idle Line cho gói tiếp theo */
    UART1_StartReceive();
  }
}

/**
 * @brief  Callback tự động khôi phục khi gặp lỗi đường truyền UART
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    (void)HAL_UART_AbortReceive(huart);
    sbus_parser_count = 0U;
    SBUS_StartReceive();
  }
  if (huart->Instance == USART1) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    (void)HAL_UART_AbortReceive(huart);
    UART1_StartReceive();
  }
}

/**
 * @brief  Chuyển đổi núm xoay Potentiometer (Kênh 7 & Kênh 8 / 200..1800) sang
 * độ lớn tốc độ (0% đến 100%)
 */
float SBUS_Knob_To_Speed(uint16_t ch_val) {
  if (ch_val < 200)
    ch_val = 200;
  if (ch_val > 1800)
    ch_val = 1800;
  return map2((float)ch_val, 200.0f, 1800.0f, 0.0f, 100.0f);
}

/**
 * @brief  Chuyển đổi núm xoay Potentiometer (200..1800) sang hệ số alpha (0.00
 * đến 1.00) Làm tròn lấy đúng 2 chữ số sau dấu phẩy (0.xx)
 */
float SBUS_Knob_To_Alpha(uint16_t ch_val) {
  if (ch_val < 200)
    ch_val = 200;
  if (ch_val > 1800)
    ch_val = 1800;
  float raw = map2((float)ch_val, 200.0f, 1800.0f, 0.0f, 1.0f);
  return (float)((uint16_t)(raw * 100.0f + 0.5f)) / 100.0f;
}

/**
 * @brief  Bộ lọc EMA siêu nhạy chống rung giật số và bám sát chuyển động xoay
 * núm
 * @param  current_alpha: Giá trị alpha hiện tại (0.00 đến 1.00)
 * @param  ch_val: Giá trị kênh SBUS đọc về (200..1800)
 * @retval Giá trị alpha mượt mà, phản hồi tức thì (nhạy) và đứng im khi không
 * xoay
 */
float Filter_Alpha(float current_alpha, uint16_t ch_val) {
  if (ch_val < 200)
    ch_val = 200;
  if (ch_val > 1800)
    ch_val = 1800;

  /* Giá trị mục tiêu từ kênh SBUS */
  float target = (float)(ch_val - 200) / 1600.0f;

  /* Lọc EMA bám nhanh (trọng số 0.8: bám theo tay tức thì trong ~15ms, vừa
   * triệt tiêu nhiễu rung) */
  float filtered = current_alpha + 0.8f * (target - current_alpha);

  /* Làm tròn lấy 2 chữ số sau dấu phẩy */
  return (float)((uint16_t)(filtered * 100.0f + 0.5f)) / 100.0f;
}

/**
 * @brief  Hàm tính toán gia tốc tăng/giảm tốc độ theo thời gian
 * @param  current_speed: Tốc độ hiện tại (%)
 * @param  target_speed : Tốc độ cài đặt cần đạt tới (%)
 * @param  time_ms      : Khoảng thời gian (ms) để tăng từ AUTO_MIN_SPEED (0%) đến target_speed
 * @retval Tốc độ mới sau khi áp dụng gia tốc (%)
 */
float Speed_Ramp(float current_speed, float target_speed, float time_ms) {
  if (time_ms <= 0.0f) {
    return target_speed;
  }

  static uint32_t last_tick = 0;
  uint32_t now = HAL_GetTick();
  float dt = (float)(now - last_tick);
  last_tick = now;

  /* Giới hạn dt hợp lệ nếu hàm vừa được gọi lại sau thời gian dài */
  if (dt <= 0.0f || dt > 50.0f) {
    dt = 5.0f; /* Thời gian chu kỳ mặc định theo HAL_Delay(5) */
  }

  /* Bước tăng/giảm tốc độ trong khoảng thời gian dt:
   * Khoảng chênh lệch từ AUTO_MIN_SPEED (0%) đến target_speed diễn ra đúng trong time_ms */
  float speed_delta = fabsf(target_speed - AUTO_MIN_SPEED);
  if (speed_delta <= 0.0f) {
    return target_speed;
  }

  float rate = speed_delta / time_ms;
  float step = rate * dt;

  if (current_speed < target_speed) {
    current_speed += step;
    if (current_speed > target_speed) {
      current_speed = target_speed;
    }
  } else if (current_speed > target_speed) {
    current_speed -= step;
    if (current_speed < target_speed) {
      current_speed = target_speed;
    }
  }

  return current_speed;
}

/**
 * @brief  Hàm ánh xạ tuyến tính float
 */
float map2(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) {
    return out_min;
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief  Tính toán động học vi sai (Kinematics) và cấp tốc độ ra 2 động cơ
 * (-100 đến 100%)
 * @param  u: Vận tốc dài (Linear velocity) - vector đi thẳng/lùi
 * @param  r: Vận tốc góc (Angular velocity) - vector quay trái/phải
 * @retval None
 */
void Kinematic_Cal(float u, float r) {
  /* Công thức động học vi sai (đã bỏ hệ số alpha ở từng bánh):
   * omega1: vận tốc góc bánh phải (Motor 1)
   * omega2: vận tốc góc bánh trái (Motor 2)
   */
  omega1 = (1.0f / a) * (u + r * d);
  omega2 = (1.0f / a) * (u -  r * d);

  /* Ánh xạ trực tiếp sang dải PWM (-100% đến 100%) */
  int16_t right_speed =
      -(int16_t)map2(omega1, -100.0f, 100.0f, -100.0f, 100.0f);
  int16_t left_speed = (int16_t)map2(omega2, -100.0f, 100.0f, -100.0f, 100.0f);

  /* Cấp tốc độ trực tiếp ra 2 động cơ */
  Motor1_SetSpeed(right_speed);
  Motor2_SetSpeed(left_speed);
}

/**
 * @brief  Điều khiển Động cơ 1 (Bánh phải)
 * @param  pwm_val: -100 đến 100 (%)
 *                  - pwm_val >= 0: Quay thuận (DIR1 = HIGH), xung PA0
 * (TIM2_CH1)
 *                  - pwm_val < 0 : Quay nghịch (DIR1 = LOW), xung PA0
 * (TIM2_CH1)
 * @retval None
 */
void Motor1_SetSpeed(int16_t pwm_val) {
  /* Giới hạn giá trị trong khoảng [-100, 100] */
  if (pwm_val > 100) {
    pwm_val = 100;
  } else if (pwm_val < -100) {
    pwm_val = -100;
  }

  /* Điều khiển chân chiều DIR 1 (PB1) */
  if (pwm_val >= 0) {
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
    pwm_val = -pwm_val; /* Lấy độ lớn cho xung PWM */
  }

  /* Tính giá trị compare tương ứng theo Period (ARR = 719 => 720 bước đếm cho
   * 100kHz) */
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U;
  uint32_t compare = (period * (uint32_t)pwm_val) / 100U;

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare);
}

/**
 * @brief  Điều khiển Động cơ 2 (Bánh trái)
 * @param  pwm_val: -100 đến 100 (%)
 *                  - pwm_val >= 0: Quay thuận (DIR2 = HIGH), xung PA1
 * (TIM2_CH2)
 *                  - pwm_val < 0 : Quay nghịch (DIR2 = LOW), xung PA1
 * (TIM2_CH2)
 * @retval None
 */
void Motor2_SetSpeed(int16_t pwm_val) {
  /* Giới hạn giá trị trong khoảng [-100, 100] */
  if (pwm_val > 100) {
    pwm_val = 100;
  } else if (pwm_val < -100) {
    pwm_val = -100;
  }

  /* Điều khiển chân chiều DIR 2 (PB0) */
  if (pwm_val >= 0) {
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
    pwm_val = -pwm_val; /* Lấy độ lớn cho xung PWM */
  }

  /* Tính giá trị compare tương ứng theo Period (ARR = 719 => 720 bước đếm cho
   * 100kHz) */
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U;
  uint32_t compare = (period * (uint32_t)pwm_val) / 100U;

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, compare);
}

/**
 * @brief  Hàm điều khiển chung / tương thích ngược
 */
void Motor_SetSpeed(int16_t pwm_val) { Motor1_SetSpeed(pwm_val); }

/**
 * @brief  Thuật toán PID bám đường (Path Tracking)
 * @param  target: Giá trị mục tiêu (mặc định 0.0f là ở giữa tâm đường)
 * @retval Tín hiệu ngõ ra điều khiển góc quay (uout_path: -100.0f đến 100.0f)
 */
float pid_path(float target) {
  /* Nếu tại trạm dừng QR (qr_stop_flag == 1): ngắt ngõ ra lái về 0 và xóa khâu tích phân */
  if (qr_stop_flag == 1) {
    ui_path = 0.0f;
    uout_path = 0.0f;
    return 0.0f;
  }

  /* Sai số: mục tiêu - độ lệch thực tế nhận từ cảm biến qua UART1 */
  err_path = target - error_path;

  /* Khâu tỉ lệ (P) */
  up_path = kp_path * err_path;

  /* Khâu tích phân (I) kèm giới hạn chống bão hòa (Anti-windup) */
  ui_path += ki_path * err_path * deltaT_path;
  if (ui_path > PID_MAX) ui_path = PID_MAX;
  else if (ui_path < PID_MIN) ui_path = PID_MIN;

  /* Khâu vi phân (D) có bộ lọc thông thấp (Derivative Filter) chống nhiễu */
  ud_path = kd_path * ((err_path - pre_err_path) / deltaT_path);
  ud_fil_path = (1.0f - alpha_fil_path) * ud_fil_pre_path + alpha_fil_path * ud_path;

  /* Lưu giá trị cho chu kỳ kế tiếp */
  ud_fil_pre_path = ud_fil_path;
  pre_err_path = err_path;

  /* Tổng ngõ ra PID (giới hạn trong khoảng [PID_MIN, PID_MAX]) */
  uout_path = up_path + ud_fil_path + ui_path;
  if (uout_path > PID_MAX) uout_path = PID_MAX;
  else if (uout_path < PID_MIN) uout_path = PID_MIN;

  return uout_path;
}

/**
 * @brief  Hàm reset các biến trạng thái bộ điều khiển PID Path
 */
void pid_reset_path(void) {
  err_path = 0.0f;
  pre_err_path = 0.0f;
  up_path = 0.0f;
  ui_path = 0.0f;
  ud_path = 0.0f;
  ud_fil_path = 0.0f;
  ud_fil_pre_path = 0.0f;
  uout_path = 0.0f;
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* Khởi động xuất xung PWM trên Timer 2: Channel 1 (PA0) và Channel 2 (PA1) */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* Bắt đầu nhận tín hiệu điều khiển SBUS qua DMA */
  SBUS_StartReceive();

  /* Bắt đầu nhận dữ liệu UART1 qua DMA Idle Line (mảng 10 byte) */
  UART1_StartReceive();

  /* Khởi động ngắt định kỳ Timer 3 để tính toán bộ điều khiển PID bám đường */
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* Đọc tín hiệu SBUS từ tay điều khiển RC */
    if (sbus_failsafe == 0 && sbus_sync_ok == 1) {
      /* 1. Lọc và cập nhật hệ số alpha1 (bánh phải) và alpha2 (bánh trái) từ
       * núm xoay (0.00 đến 1.00) */
      alpha1 = Filter_Alpha(
          alpha1,
          sbus_channel[6]); /* Kênh 7: Núm xoay điều chỉnh % tốc độ bánh phải */
      alpha2 = Filter_Alpha(
          alpha2,
          sbus_channel[7]); /* Kênh 8: Núm xoay điều chỉnh % tốc độ bánh trái */

      /* 2. Đọc trạng thái Tự động chạy thẳng (Auto_Forward) từ công tắc Kênh 5
       * (sbus_channel[4]):
       * - Gạt lên (~200 / < 600)    -> Auto_Forward = 1 (Bật chế độ tự chạy
       * thẳng)
       * - Gạt xuống (~1800 / > 1400) -> Auto_Forward = 0 (Tắt tự động, về điều
       * khiển tay)
       */
      if (sbus_channel[4] < 600) {
        Auto_Forward = 1;
      } else {
        Auto_Forward = 0;
      }

      /* 3. Điều khiển Tốc độ tiến/lùi (cmd_u) và Góc xoay (cmd_r):
       * - cmd_u: sbus_channel[1] (200 -> 1800 map sang -100 -> +100, tâm 1000)
       * - cmd_r: sbus_channel[0] (200 -> 1800 map sang -100 -> +100, tâm 1000)
       */
      /* Đọc giá trị cần gạt từ tay điều khiển RC */
      float raw_ch1 = (float)sbus_channel[1];
      if (raw_ch1 < 200.0f) raw_ch1 = 200.0f;
      else if (raw_ch1 > 1800.0f) raw_ch1 = 1800.0f;
      float rc_u = map2(raw_ch1, 200.0f, 1800.0f, -100.0f, 100.0f);
      if (rc_u >= -30.0f && rc_u <= 30.0f) {
        rc_u = 0.0f; /* Vùng chết [-30, 30] ở khoảng giữa cần gạt tiến/lùi */
      }

      float raw_ch0 = (float)sbus_channel[0];
      if (raw_ch0 < 200.0f) raw_ch0 = 200.0f;
      else if (raw_ch0 > 1800.0f) raw_ch0 = 1800.0f;
      float rc_r = - map2(raw_ch0, 200.0f, 1800.0f, -100.0f, 100.0f);
      if (rc_r >= -30.0f && rc_r <= 30.0f) {
        rc_r = 0.0f; /* Vùng chết [-30, 30] ở khoảng giữa cần gạt xoay */
      }

      /* 3.1. KIỂM TRA CHẾ ĐỘ TỰ ĐỘNG BẺ LÁI THEO PID BÁM ĐƯỜNG (Núm xoay 2 > 50%) */
      if (alpha2 > 0.5f) {
        path_pid_active = 1;

        /* Khi núm 2 > 0.5: Góc xoay cmd_r lấy trực tiếp từ ngõ ra PID (uout_path),
         * hoàn toàn KHÔNG bị giới hạn bởi các quy luật 150ms hay 400ms! */
        cmd_r = uout_path;

        if (Auto_Forward == 1) {
          float target_speed = alpha1 * 100.0f;
          is_forward = 1;

          /* Bắt đầu tăng tốc ở lần đầu tiên khi bật Auto_Forward */
          if (pre_Auto_Forward == 0) {
            cmd_u = AUTO_MIN_SPEED;
            ramp_done = 0;
            pre_Auto_Forward = 1;
          }

          /* Gia tốc tăng tốc độ đến target_speed */
          if (ramp_done == 0) {
            cmd_u = Speed_Ramp(cmd_u, target_speed, auto_accel_time_ms);
            if (cmd_u >= target_speed) {
              cmd_u = target_speed;
              ramp_done = 1;
            }
          } else {
            cmd_u = target_speed;
          }
        } else {
          /* Chế độ Lái tay: Vận tốc tiến/lùi theo cần gạt rc_u * alpha1 */
          pre_Auto_Forward = 0;
          ramp_done = 0;
          cmd_u_base = rc_u * alpha1;
          cmd_u = cmd_u_base;

          if (cmd_u_base > 0.0f) {
            is_forward = 1;
          } else {
            is_forward = 0;
          }

          /* Nếu buông tay cần tiến/lùi (xe đứng yên), ngắt cả tốc độ và góc xoay để xe đứng yên */
          if (rc_u == 0.0f) {
            cmd_u = 0.0f;
            cmd_r = 0.0f;
          }
        }

        /* KIỂM TRA TRẠM DỪNG QR (> 200x200 PIXELS) TRONG CHẾ ĐỘ TỰ ĐỘNG BÁM ĐƯỜNG:
         * Khi tầng trên gửi cờ trạm dừng (qr_stop_flag == 1),
         * lập tức dừng hẳn robot (ngắt tốc độ cmd_u = 0, góc lái cmd_r = 0).
         */
        if (qr_stop_flag == 1) {
          cmd_u = 0.0f;
          cmd_r = 0.0f;
          ramp_done = 0; /* Reset gia tốc để khi tiếp tục chạy sẽ khởi động êm */
          ui_path = 0.0f; /* Xóa tích phân PID */
        }

        /* Nạp lại cờ sẵn sàng cho các đợt bẻ lái tay khi tắt núm xoay PID */
        auto_turn_armed = 1;
        auto_turn_active = 0;
        manual_turn_armed = 1;
        manual_turn_active = 0;
      } else {
        /* TẮT CHẾ ĐỘ PID BÁM ĐƯỜNG: Reset trạng thái PID khi vừa vặn núm về <= 0.5 */
        if (path_pid_active == 1) {
          pid_reset_path();
          path_pid_active = 0;
        }

        /* === CHẾ ĐỘ ĐIỀU KHIỂN THEO CẦN GẠT TAY VỚI QUY LUẬT 150ms VÀ 400ms === */
        if (Auto_Forward == 1) {
          float target_speed = alpha1 * 100.0f;
          is_forward = 1; /* Ở chế độ Auto thì xe luôn luôn ở trạng thái tiến */

          /* Bắt đầu tăng tốc ở lần đầu tiên khi bật Auto_Forward */
          if (pre_Auto_Forward == 0) {
            cmd_u = AUTO_MIN_SPEED; /* Khởi tạo tốc độ bắt đầu từ 0% */
            ramp_done = 0;
            auto_turn_armed = 1;
            auto_turn_active = 0;
            pre_Auto_Forward = 1;
          }

          /* Xử lý xoay ngang trong chế độ Auto_Forward:
           * - Nhả cần xoay về giữa (rc_r == 0): Lập tức dừng xoay (kể cả chưa hết 150ms).
           * - Bắt đầu gạt cần xoay (rc_r != 0): Chỉ kích hoạt khi đã nghỉ đủ turn_debounce_ms (100ms).
           */
          if (rc_r == 0.0f) {
            /* Chỉ nạp lại cờ sẵn sàng sau khi đã nghỉ đủ turn_debounce_ms (chống spam cần xoay) */
            if ((HAL_GetTick() - auto_turn_end_tick) >= turn_debounce_ms) {
              auto_turn_armed = 1;  /* Nhả về giữa và đã qua thời gian nghỉ: sẵn sàng cho lần gạt sau */
            }
            if (auto_turn_active == 1) {
              auto_turn_active = 0; /* Lập tức dừng xoay */
              auto_turn_end_tick = HAL_GetTick(); /* Bắt đầu đếm thời gian nghỉ chống spam */
            }
            cmd_r = 0.0f;
          } else if (auto_turn_armed == 1 && (HAL_GetTick() - auto_turn_end_tick) >= turn_debounce_ms) {
            auto_turn_armed = 0;  /* Khóa lại, chỉ xoay 1 đợt 150ms */
            auto_turn_active = 1; /* Bắt đầu xoay */
            auto_turn_start_tick = HAL_GetTick();
          }

          /* Thực hiện xoay nếu đang trong thời gian cho phép (< 150ms) */
          if (auto_turn_active == 1) {
            if ((HAL_GetTick() - auto_turn_start_tick) < auto_turn_duration_ms) {
              /* VỪA CHẠY VỪA LƯỢN: Giảm tốc tịnh tiến xuống 2/3 để ôm cua an toàn */
              cmd_u = target_speed * (2.0f / 3.0f);
              cmd_r = rc_r * alpha1 * (2.0f / 3.0f);        /* Xoay theo cần gạt tay (nhân hệ số 2/3 để giảm tốc góc) */
            } else {
              /* Hết 150ms: Tự động hết thời gian xoay dù tay vẫn đang giữ cần gạt */
              auto_turn_active = 0;
              auto_turn_end_tick = HAL_GetTick();           /* Bắt đầu đếm cooldown chống spam */
              cmd_r = 0.0f;
            }
          }

          /* Khi không xoay (hoặc vừa hết 150ms): Chạy thẳng bình thường với tốc độ cài đặt */
          if (auto_turn_active == 0) {
            cmd_r = 0.0f;          /* Ngừng xoay khi chạy thẳng */

            /* Chỉ gia tốc 1 lần duy nhất lúc vừa bật Auto_Forward */
            if (ramp_done == 0) {
              cmd_u = Speed_Ramp(cmd_u, target_speed, auto_accel_time_ms);
              if (cmd_u >= target_speed) {
                cmd_u = target_speed;
                ramp_done = 1; /* Chạy xong gia tốc 1 lần duy nhất -> các lần sau không gia tốc nữa */
              }
            } else {
              cmd_u = target_speed; /* Đã xong gia tốc: chạy thẳng trực tiếp với tốc độ cài đặt */
            }
          }
        } else {
          /* Tắt Auto_Forward: reset cờ để lần sau bật lại sẽ tính gia tốc từ đầu */
          pre_Auto_Forward = 0;
          ramp_done = 0;
          auto_turn_armed = 1;
          auto_turn_active = 0;

          /* Chế độ lái tay thủ công (không bật Auto_Forward):
           * 1. Khi thả cần chạy thẳng (rc_u == 0):
           *    - Xoay toàn diện tự do (cmd_u = 0, cmd_r = rc_r * alpha1).
           * 2. Khi đang gạt chạy thẳng (rc_u != 0):
           *    - Xe duy trì chạy thẳng (cmd_u = rc_u * alpha1), KHÔNG dừng lại để xoay.
           *    - Nếu gạt ngang: chỉ xoay trong 150ms như Auto_Forward (cmd_r = rc_r * alpha1).
           *      Hết 150ms thì tự động ngắt xoay (cmd_r = 0), tiếp tục chạy thẳng.
           *    - Nếu nhả cần xoay về giữa (rc_r == 0) trong 150ms: lập tức dừng xoay (cmd_r = 0) và nạp lại cờ.
           *    - Chỉ được xoay tiếp 150ms khi đã nhả cần xoay về giữa rồi gạt lại.
           */
          if (rc_u == 0.0f) {
            /* Không gạt chạy thẳng -> Cho phép xoay toàn diện không giới hạn thời gian */
            is_forward = 0;
            cmd_u = 0.0f;
            cmd_u_base = 0.0f;
            cmd_r = rc_r * alpha1;

            /* Nạp lại cờ sẵn sàng cho đợt xoay 150ms khi chạy thẳng */
            manual_turn_armed = 1;
            manual_turn_active = 0;
          } else {
            /* Đang gạt chạy thẳng/lùi: Tính tốc độ cơ sở từ cần gạt */
            cmd_u_base = rc_u * alpha1;

            /* Cập nhật cờ hướng chạy */
            if (cmd_u_base > 0.0f) {
              is_forward = 1;  /* Đang tiến */
            } else {
              is_forward = 0;  /* Đang lùi */
            }

            /* Xử lý gạt xoay ngang trong lúc chạy thẳng (chỉ xoay tối đa 150ms) */
            if (rc_r == 0.0f) {
              /* Chỉ nạp lại cờ sẵn sàng sau khi đã nghỉ đủ 50ms (chống spam cần xoay) */
              if ((HAL_GetTick() - manual_turn_end_tick) >= turn_debounce_ms) {
                manual_turn_armed = 1;  /* Nhả về giữa và đã qua 50ms: sẵn sàng cho lần gạt sau */
              }
              if (manual_turn_active == 1) {
                manual_turn_active = 0; /* Lập tức dừng xoay */
                manual_turn_end_tick = HAL_GetTick(); /* Bắt đầu đếm thời gian nghỉ 50ms */
              }
              cmd_r = 0.0f;
              cmd_u = cmd_u_base;     /* Chạy thẳng đủ 100% tốc độ cài đặt */
            } else if (manual_turn_armed == 1 && (HAL_GetTick() - manual_turn_end_tick) >= turn_debounce_ms) {
              manual_turn_armed = 0;  /* Khóa lại, chỉ xoay 1 đợt 150ms */
              manual_turn_active = 1;
              manual_turn_start_tick = HAL_GetTick();
            }

            if (manual_turn_active == 1) {
              if ((HAL_GetTick() - manual_turn_start_tick) < auto_turn_duration_ms) {
                /* Trong 150ms liếc hướng: Giảm tốc tịnh tiến 2/3 để ôm cua an toàn */
                cmd_u = cmd_u_base * (2.0f / 3.0f);

                /* Bẻ lái: kiểm tra cờ is_forward cực kỳ trực quan */
                if (is_forward == 0) {
                  cmd_r = -rc_r * alpha1 * (2.0f / 3.0f); /* Lùi xe: đảo ngược hướng xoay và giảm tốc 2/3 */
                } else {
                  cmd_r = rc_r * alpha1 * (2.0f / 3.0f);  /* Tiến xe: hướng xoay bình thường giảm tốc 2/3 */
                }
              } else {
                manual_turn_active = 0; /* Hết 150ms: Tự động hết thời gian xoay */
                manual_turn_end_tick = HAL_GetTick(); /* Bắt đầu đếm 50ms cooldown chống spam */
                cmd_r = 0.0f;
                cmd_u = cmd_u_base;     /* Trả lại đủ tốc độ chạy thẳng */
              }
            } else {
              /* Hết 150ms mà chưa nhả cần về 0: Khóa xoay, chỉ chạy thẳng với đủ tốc độ */
              cmd_r = 0.0f;
              cmd_u = cmd_u_base;
            }
          }
        }
      }

      /* Giới hạn góc lái cmd_r trong khoảng [-100, 100] */
      if (cmd_r > 100.0f) {
        cmd_r = 100.0f;
      } else if (cmd_r < -100.0f) {
        cmd_r = -100.0f;
      }

      /* 4. Điều khiển Bơm (sbus_channel[5]):
       * - Bỏ qua nếu trước đó channel 4..7 đều bằng 200 (trạng thái khởi động / dummy frame ban đầu)
       * - Khi giá trị trước đó < 600 và hiện tại chuyển về vị trí giữa (~1000, [800..1200]) -> Bật bơm (Pump_control = 1)
       * - Khi gạt xuống (> 1400) -> Tắt bơm (Pump_control = 0)
       */
      uint8_t prev_ch4_to_7_all_200 = (pre_value_ch4 <= 250 && pre_value_ch5 <= 250 &&
                                       pre_value_ch6 <= 250 && pre_value_ch7 <= 250);

      if ((sbus_channel[5] >= 800 && sbus_channel[5] <= 1200) && (pre_value_ch5 < 600) && !prev_ch4_to_7_all_200) {
        Pump_control = 1;
      } else if (sbus_channel[5] > 1400) {
        Pump_control = 0;
      }
      pre_value_ch4 = sbus_channel[4];
      pre_value_ch5 = sbus_channel[5];
      pre_value_ch6 = sbus_channel[6];
      pre_value_ch7 = sbus_channel[7];
    } else {
      /* Mất sóng hoặc tắt tay điều khiển -> dừng khẩn cấp, tắt bơm và tắt tự
       * động */
      cmd_u = 0.0f;
      cmd_u_base = 0.0f;
      cmd_r = 0.0f;
      Auto_Forward = 0;
      pre_Auto_Forward = 0;
      ramp_done = 0;
      auto_turn_armed = 1;
      auto_turn_active = 0;
      manual_turn_armed = 1;
      manual_turn_active = 0;
      Pump_control = 0;
      is_forward = 0;
      path_pid_active = 0;
      pid_reset_path();
      control_axis = AXIS_NONE;
      previous_control_axis = AXIS_NONE;
      pre_rc_u = 0.0f;
      pre_rc_r = 0.0f;
    }

    /* 5. Điều khiển 4 Relay (Bơm thuốc) theo biến Pump_control */
       if (Pump_control == 1) {
         HAL_GPIO_WritePin(GPIOB,
                           Relay1_Pin | Relay2_Pin | Relay3_Pin | Relay4_Pin,
                           GPIO_PIN_SET);
       } else {
         HAL_GPIO_WritePin(GPIOB,
                           Relay1_Pin | Relay2_Pin | Relay3_Pin | Relay4_Pin,
                           GPIO_PIN_RESET);
       }
    /* 5. Điều khiển 4 Relay (Bơm thuốc) theo biến Pump_control */
    // if (Pump_control == 1) {
    //   HAL_GPIO_WritePin(GPIOB, Relay1_Pin, GPIO_PIN_SET);
    // } else {
    //   HAL_GPIO_WritePin(GPIOB, Relay1_Pin, GPIO_PIN_RESET);
    // }

    /* 6. HÀNH VI TRẠM DỪNG QR (> 200x200 PIXELS) TOÀN CỤC:
     * Khi nhận cờ dừng QR (qr_stop_flag == 1) trong khi đang bật chế độ tự động
     * (bật PID bám đường hoặc bật Auto_Forward), cưỡng bức dừng xe tuyệt đối.
     */
    if (qr_stop_flag == 1 && (path_pid_active == 1 || Auto_Forward == 1)) {
      cmd_u = 0.0f;
      cmd_r = 0.0f;
      ramp_done = 0;
      ui_path = 0.0f;
    }

    /* Tính toán động học vi sai và băm xung ra 2 bánh */
    Kinematic_Cal(cmd_u, cmd_r);
    HAL_Delay(5);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR2_Pin|DIR1_Pin|Relay1_Pin|Relay2_Pin
                          |Relay3_Pin|Relay4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR2_Pin DIR1_Pin Relay1_Pin Relay2_Pin
                           Relay3_Pin Relay4_Pin */
  GPIO_InitStruct.Pin = DIR2_Pin|DIR1_Pin|Relay1_Pin|Relay2_Pin
                          |Relay3_Pin|Relay4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Callback ngắt chu kỳ Timer (Period Elapsed)
  * @param  htim: con trỏ cấu trúc Timer
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3 || htim->Instance == TIM2) {
    /* Tính toán PID bám đường theo chu kỳ ngắt Timer */
    pid_path(target_path);
  }
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
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
