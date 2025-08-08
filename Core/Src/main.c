/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "stm32f0xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { MODE_IDLE = 0, MODE_1 = 1, MODE_2 = 2, MODE_3 = 3 } Mode_t;
typedef enum { SPARK_NEW = 0, SPARK_CLEAR = 1 } SparkState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Pull-ups are fitted: pin reads 1 when NOT pressed, 0 when pressed */
#define BTN_LEVEL(port,pin)  (LL_GPIO_IsInputPinSet((port),(pin)) ? 1u : 0u)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
/* ---------- global state ---------- */
volatile Mode_t  g_mode = MODE_IDLE;   /* start idle: all LEDs off (per spec) */
volatile uint8_t g_idx  = 0;           /* LED index 0..7 */
volatile int8_t  g_dir  = +1;          /* +1 forward, -1 backward */
volatile uint8_t g_fast = 0;           /* 0 => 1s, 1 => 0.5s (base speed for modes 1/2) */

volatile SparkState_t g_sp_state = SPARK_NEW;
volatile uint8_t      g_sp_mask  = 0;  /* sparkle current LED pattern */

uint32_t g_base_counts = 1000;         /* 1s worth of TIM16 counts (from Init) */

/* button edge memories (pressed = 0 because of pull-ups) */
static uint8_t b0_old = 1, b1_old = 1, b2_old = 1, b3_old = 1;

static void write_leds(uint8_t mask);

static inline uint8_t rand8(void) {
  /* simple LCG for non-critical randomness */
  static uint32_t s = 0xC0FFEEu;
  s = s*1664525u + 1013904223u;
  return (uint8_t)(s >> 24);
}

static inline void apply_base_period(void) {
  /* Modes 1/2 use fixed base periods 1.0s or 0.5s at 1 kHz timer tick */
  uint32_t counts = g_fast ? (g_base_counts >> 1) : g_base_counts;  /* 1000 or 500 */
  if (counts < 2) counts = 2;              /* avoid ARR=0/1 corner cases */
  __HAL_TIM_SET_AUTORELOAD(&htim16, (uint16_t)(counts - 1));
  __HAL_TIM_SET_COUNTER(&htim16, 0);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void write_leds(uint8_t mask)
{
  (mask & (1u<<0)) ? LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin)
                   : LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
  (mask & (1u<<1)) ? LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin)
                   : LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
  (mask & (1u<<2)) ? LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin)
                   : LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
  (mask & (1u<<3)) ? LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin)
                   : LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
  (mask & (1u<<4)) ? LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin)
                   : LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
  (mask & (1u<<5)) ? LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin)
                   : LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
  (mask & (1u<<6)) ? LL_GPIO_SetOutputPin(LED6_GPIO_Port, LED6_Pin)
                   : LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
  (mask & (1u<<7)) ? LL_GPIO_SetOutputPin(LED7_GPIO_Port, LED7_Pin)
                   : LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM16_Init();

  /* USER CODE BEGIN 2 */
  write_leds(0x00);                                    /* start with all LEDs OFF (spec) */
  g_base_counts = (uint32_t)htim16.Init.Period + 1u;   /* PSC=7999 → 1kHz, Period=999 → 1s */
  HAL_TIM_Base_Start_IT(&htim16);                      /* start timer in interrupt mode   */
  apply_base_period();                                 /* preload base period for modes 1/2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* PA0 toggles base delay 1.0s <-> 0.5s by changing TIM16 ARR (Task 4). */
    uint8_t b0 = BTN_LEVEL(Button0_GPIO_Port, Button0_Pin);
    if (b0_old == 1 && b0 == 0) {
      g_fast ^= 1u;
      if (g_mode == MODE_1 || g_mode == MODE_2) {
        apply_base_period();                           /* only affects modes 1/2 per brief */
      }
    }
    b0_old = b0;
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0) {}
  LL_RCC_HSI_Enable();
  while (LL_RCC_HSI_IsReady() != 1) {}
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {}
  LL_SetSystemCoreClock(8000000);
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;             /* 8 MHz / 8000 = 1 kHz tick */
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;                /* default 1.0 s */
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK) { Error_Handler(); }
  NVIC_EnableIRQ(TIM16_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /* Clear LEDs */
  LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
  LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
  LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /* Buttons: pull-up inputs */
  GPIO_InitStruct.Pin = Button0_Pin; GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT; GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button0_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Button1_Pin; LL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Button2_Pin; LL_GPIO_Init(Button2_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Button3_Pin; LL_GPIO_Init(Button3_GPIO_Port, &GPIO_InitStruct);

  /* LEDs: push-pull outputs */
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;

  GPIO_InitStruct.Pin = LED0_Pin; LL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LED1_Pin; LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LED2_Pin; LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LED3_Pin; LL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LED4_Pin; LL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LED5_Pin; LL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LED6_Pin; LL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LED7_Pin; LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* We use HAL's PeriodElapsed callback. The vector ISR calls HAL_TIM_IRQHandler. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM16) return;

  /* Mode select buttons PA1/PA2/PA3 – action on falling edge (pressed) */
  uint8_t b1 = BTN_LEVEL(Button1_GPIO_Port, Button1_Pin);
  uint8_t b2 = BTN_LEVEL(Button2_GPIO_Port, Button2_Pin);
  uint8_t b3 = BTN_LEVEL(Button3_GPIO_Port, Button3_Pin);

  if (b1_old == 1 && b1 == 0) { g_mode = MODE_1; g_idx = 0; g_dir = +1; apply_base_period(); }
  if (b2_old == 1 && b2 == 0) { g_mode = MODE_2; g_idx = 0; g_dir = +1; apply_base_period(); }
  if (b3_old == 1 && b3 == 0) { g_mode = MODE_3; g_sp_state = SPARK_NEW; /* next tick will start sparkle */ }
  b1_old = b1; b2_old = b2; b3_old = b3;

  switch (g_mode) {

  case MODE_IDLE:
    /* nothing – LEDs remain off until a mode is chosen */
    break;

  case MODE_1: { /* single LED back/forth without repeating ends (Figure 1) */
    uint8_t mask = (1u << g_idx);
    write_leds(mask);
    if (g_dir > 0) { if (g_idx == 7) { g_dir = -1; g_idx = 6; } else g_idx++; }
    else            { if (g_idx == 0) { g_dir = +1; g_idx = 1; } else g_idx--; }
  } break;

  case MODE_2: { /* inverse back/forth (Figure 2) */
    uint8_t mask = (uint8_t)(~(1u << g_idx)) & 0xFFu;
    write_leds(mask);
    if (g_dir > 0) { if (g_idx == 7) { g_dir = -1; g_idx = 6; } else g_idx++; }
    else            { if (g_idx == 0) { g_dir = +1; g_idx = 1; } else g_idx--; }
  } break;

  case MODE_3: { /* Sparkle (Figure 3) – continuous */
    if (g_sp_state == SPARK_NEW) {
      g_sp_mask = rand8();                         /* 0..255 pattern */
      write_leds(g_sp_mask);
      /* hold 100..1500 ms */
      uint16_t hold_ms = 100u + (uint16_t)(rand8() % 1401u);
      __HAL_TIM_SET_AUTORELOAD(&htim16, (hold_ms >= 2 ? hold_ms : 2) - 1u);
      __HAL_TIM_SET_COUNTER(&htim16, 0);
      g_sp_state = SPARK_CLEAR;
    } else { /* clearing phase: turn off exactly one '1' bit per tick */
      if (g_sp_mask) {
        for (uint8_t i = 0; i < 8; ++i) {          /* left-to-right clear */
          if (g_sp_mask & (1u << i)) { g_sp_mask &= (uint8_t)~(1u << i); break; }
        }
        write_leds(g_sp_mask);
        /* 1..100 ms gap between clears */
        uint16_t gap_ms = (uint16_t)((rand8() % 100u) + 1u);
        __HAL_TIM_SET_AUTORELOAD(&htim16, (gap_ms >= 2 ? gap_ms : 2) - 1u);
        __HAL_TIM_SET_COUNTER(&htim16, 0);
      } else {
        /* after last LED off, wait a small random gap and start a NEW sparkle */
        uint16_t gap_ms = (uint16_t)((rand8() % 100u) + 1u);
        __HAL_TIM_SET_AUTORELOAD(&htim16, (gap_ms >= 2 ? gap_ms : 2) - 1u);
        __HAL_TIM_SET_COUNTER(&htim16, 0);
        g_sp_state = SPARK_NEW;
      }
    }
  } break;

  default: break;
  }
}

/* Keep the IRQ handler minimal – it just feeds HAL, which calls our callback above. */
void TIM16_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim16);
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
