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
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "bme280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// My sample type
typedef struct {
    uint32_t time_ms;      // e.g., HAL_GetTick()
    float    thermistor_deg_c;  // thermistor temp
    float    ldr_pct;      // photoresistor %
    float    bme_temp_c;   // BME280 compensated temperature (deg C)
    float    bme_press_hpa; // BME280 pressure (hPa)
    float    bme_hum_pct;  // BME280 relative humidity (%)
    bool     bme_ok;       // true if last BME280 read in this sample succeeded
} Sample;

// Ring buffer
typedef struct{
	Sample buf[512u];
	uint16_t head;
	uint16_t tail;
	uint16_t count;
} Ring;

typedef struct {
  uint32_t bme_i2c_fail;
  uint32_t bme_read_fail;
  uint32_t ring_overwrite;  // or keep using ring.overwrites only
} ErrorStats;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Voltage_25 0.76f
#define AvgSlope 0.0025f
#define VrefInt 1.21f
#define R_FIXED 10000
#define BETA 3950
#define R0 10000

#define RB_CAPACITY 512u
#define RB_MASK (RB_CAPACITY - 1u)

/* BME280 7-bit I2C address: use PRIM (0x76) or SEC (0x77) per module SDO strap */
#define BME280_I2C_ADDR_7BIT  BME280_I2C_ADDR_PRIM

#define BME280_I2C_TIMEOUT_MS  100u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t interrupt_flag = 0;
static Ring ringbuffer;
Sample sample;

uint8_t rx_char;
char rx_buffer[64];
uint8_t rx_index = 0;

static uint8_t s_bme280_i2c_addr_7bit = BME280_I2C_ADDR_7BIT;
static struct bme280_dev s_bme280_dev;
static struct bme280_settings s_bme280_settings;
static bool s_bme280_ready;
static uint32_t s_bme280_meas_delay_us;
/* Consecutive BME read failures in runtime path (saturating). */
static uint8_t s_bme_consecutive_fail;
/* Tick timestamp of the last re-init attempt (for throttle window). */
static uint32_t s_bme_last_reinit_ms;
/* Count of failed re-init attempts where s_bme280_ready stayed false. */
static uint8_t s_bme_reinit_fail_count;
/* Fault latch: once set, watchdog refresh is intentionally stopped. */
static bool s_fault_latched;
static ErrorStats error_stats;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void bme280_app_init(void);
static void read_bme280_into_sample(Sample *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Send text to TeraTerm
static void uart_print(const char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)strlen(msg), 100);
}

/* Bosch BME280 driver I2C glue: intf_ptr points to 7-bit address byte */
static int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  const uint8_t *addr7 = (const uint8_t *)intf_ptr;
  if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)((uint16_t)(*addr7) << 1u), reg_addr,
                        I2C_MEMADD_SIZE_8BIT, reg_data, (uint16_t)len,
                        BME280_I2C_TIMEOUT_MS) != HAL_OK) {
    error_stats.bme_i2c_fail++;
    return BME280_E_COMM_FAIL;
  }
  return BME280_INTF_RET_SUCCESS;
}

static int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  const uint8_t *addr7 = (const uint8_t *)intf_ptr;
  if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)((uint16_t)(*addr7) << 1u), reg_addr,
                         I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data, (uint16_t)len,
                         BME280_I2C_TIMEOUT_MS) != HAL_OK) {
    error_stats.bme_i2c_fail++;
    return BME280_E_COMM_FAIL;
  }
  return BME280_INTF_RET_SUCCESS;
}

/* Driver expects microsecond delay; HAL_Delay is ms — round up, min 1 ms */
static void bme280_delay_us(uint32_t period, void *intf_ptr)
{
  (void)intf_ptr;
  uint32_t ms = (period + 999u) / 1000u;
  if (ms == 0u) {
    ms = 1u;
  }
  HAL_Delay(ms);
}

/* Configure BME280 driver callbacks/settings and mark readiness state. */
static void bme280_app_init(void)
{
  int8_t rslt;
  uint8_t settings_sel;

  memset(&s_bme280_dev, 0, sizeof(s_bme280_dev));
  memset(&s_bme280_settings, 0, sizeof(s_bme280_settings));

  s_bme280_dev.intf = BME280_I2C_INTF;
  s_bme280_dev.intf_ptr = &s_bme280_i2c_addr_7bit;
  s_bme280_dev.read = bme280_i2c_read;
  s_bme280_dev.write = bme280_i2c_write;
  s_bme280_dev.delay_us = bme280_delay_us;

  rslt = bme280_init(&s_bme280_dev);
  if (rslt < 0) {
    s_bme280_ready = false;
    uart_print("BME280 init failed (check I2C wiring / address)\r\n");
    return;
  }

  s_bme280_settings.osr_h = BME280_OVERSAMPLING_1X;
  s_bme280_settings.osr_p = BME280_OVERSAMPLING_1X;
  s_bme280_settings.osr_t = BME280_OVERSAMPLING_1X;
  s_bme280_settings.filter = BME280_FILTER_COEFF_OFF;
  s_bme280_settings.standby_time = BME280_STANDBY_TIME_1000_MS;

  settings_sel = (uint8_t)(BME280_SEL_OSR_HUM | BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP |
                           BME280_SEL_FILTER | BME280_SEL_STANDBY);
  rslt = bme280_set_sensor_settings(settings_sel, &s_bme280_settings, &s_bme280_dev);
  if (rslt < 0) {
    s_bme280_ready = false;
    uart_print("BME280 set_sensor_settings failed\r\n");
    return;
  }

  rslt = bme280_cal_meas_delay(&s_bme280_meas_delay_us, &s_bme280_settings);
  if (rslt < 0) {
    s_bme280_ready = false;
    uart_print("BME280 cal_meas_delay failed\r\n");
    return;
  }

  s_bme280_ready = true;
  uart_print("BME280 ready\r\n");
}

/* Read one BME sample and apply retry/re-init/escalation policy on failures. */
static void read_bme280_into_sample(Sample *s)
{
  struct bme280_data data;
  int8_t rslt;
  uint32_t now_ms; /* Snapshot tick once to keep all checks coherent. */

  s->bme_ok = false;
  s->bme_temp_c = 0.0f;
  s->bme_press_hpa = 0.0f;
  s->bme_hum_pct = 0.0f;

  now_ms = HAL_GetTick();

  /* If init is currently not ready, treat this as a read failure candidate. */
  if (!s_bme280_ready) {
    /* Saturating increment prevents wraparound from 255 -> 0. */
    if (s_bme_consecutive_fail < 255u) {
      s_bme_consecutive_fail++;
    }
    /* Retry init after N consecutive failures and 5 s throttle window. */
    if ((s_bme_consecutive_fail >= 3u) && ((now_ms - s_bme_last_reinit_ms) >= 5000u)) {
      s_bme_last_reinit_ms = now_ms;
      bme280_app_init();
      /* Re-init is considered failed only if driver remains not-ready. */
      if (!s_bme280_ready) {
        if (s_bme_reinit_fail_count < 255u) {
          s_bme_reinit_fail_count++;
        }
        /* Escalate after 3 failed re-init attempts: stop feeding IWDG. */
        if (s_bme_reinit_fail_count >= 3u) {
          s_fault_latched = true;
        }
      } else {
        /* Re-init recovered sensor; clear escalation counter. */
        s_bme_reinit_fail_count = 0u;
      }
      /* Re-init attempt consumed this failure streak window. */
      s_bme_consecutive_fail = 0u;
    }
    return;
  }

  /* Forced mode trigger for one-shot measurement. */
  rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &s_bme280_dev);
  if (rslt < 0) {
    if (s_bme_consecutive_fail < 255u) {
      s_bme_consecutive_fail++;
    }
    /* Same re-init policy for mode-set failure path. */
    if ((s_bme_consecutive_fail >= 3u) && ((now_ms - s_bme_last_reinit_ms) >= 5000u)) {
      s_bme_last_reinit_ms = now_ms;
      bme280_app_init();
      if (!s_bme280_ready) {
        if (s_bme_reinit_fail_count < 255u) {
          s_bme_reinit_fail_count++;
        }
        if (s_bme_reinit_fail_count >= 3u) {
          s_fault_latched = true;
        }
      } else {
        s_bme_reinit_fail_count = 0u;
      }
      s_bme_consecutive_fail = 0u;
    }
    return;
  }

  /* Wait sensor conversion time computed from current BME settings. */
  s_bme280_dev.delay_us(s_bme280_meas_delay_us, s_bme280_dev.intf_ptr);

  /* Read compensated temperature/pressure/humidity data. */
  rslt = bme280_get_sensor_data(BME280_ALL, &data, &s_bme280_dev);
  if (rslt < 0) {
    if (s_bme_consecutive_fail < 255u) {
      s_bme_consecutive_fail++;
    }
    /* Same re-init policy for data-read failure path. */
    if ((s_bme_consecutive_fail >= 3u) && ((now_ms - s_bme_last_reinit_ms) >= 5000u)) {
      s_bme_last_reinit_ms = now_ms;
      bme280_app_init();
      if (!s_bme280_ready) {
        if (s_bme_reinit_fail_count < 255u) {
          s_bme_reinit_fail_count++;
        }
        if (s_bme_reinit_fail_count >= 3u) {
          s_fault_latched = true;
        }
      } else {
        s_bme_reinit_fail_count = 0u;
      }
      s_bme_consecutive_fail = 0u;
    }
    return;
  }

  s->bme_temp_c = (float)data.temperature;
  s->bme_press_hpa = (float)(data.pressure / 100.0);
  s->bme_hum_pct = (float)data.humidity;
  s->bme_ok = true;
  /* A good sample clears both soft and escalated failure streaks. */
  s_bme_consecutive_fail = 0u;
  s_bme_reinit_fail_count = 0u;
}

// Read one ADC conversion result (polling). This returns HAL_OK / HAL_TIMEOUT / HAL_ERROR.
static HAL_StatusTypeDef adc_poll_get(uint16_t *out)
{
  if (HAL_ADC_PollForConversion(&hadc1, 200) != HAL_OK) {
    return HAL_TIMEOUT;
  }
  *out = (uint16_t)HAL_ADC_GetValue(&hadc1);
  return HAL_OK;
}

/* =======================
   Critical section helpers
   ======================= */
// Works on Cortex-M: PRIMASK disable/enable interrupts.
static inline uint32_t rb_enter_critical(void) {
    uint32_t primask;
    __asm volatile ("MRS %0, PRIMASK" : "=r" (primask) );
    __asm volatile ("CPSID i");
    return primask;
}

static inline void rb_exit_critical(uint32_t primask) {
    __asm volatile ("MSR PRIMASK, %0" :: "r" (primask) );
}

static inline void rb_init(Ring *rb){
	/* Full reset of ring metadata and sample storage. */
	memset(rb, 0, sizeof(*rb));
}
static inline void sample_init(Sample *sample){
	/* Clear current sample cache used by command/status output. */
	memset(sample, 0, sizeof(*sample));
}

/* =======================
   Push (overwrite oldest if full)
   Producer: typically ISR
   ======================= */
static inline void rb_push_overwrite(Ring *rb, const Sample *sample){
	uint32_t key = rb_enter_critical();

	rb->buf[rb->head] = *sample;
	rb->head = (rb->head + 1u) & RB_MASK;

	if(rb->count < RB_CAPACITY){
		rb->count++;
	} else{
		rb->tail = (rb->tail +1u) & RB_MASK;
		/* Track data loss when producer outruns consumer. */
		error_stats.ring_overwrite++;
	}
	rb_exit_critical(key);
}
/* =======================
   Pop
   Consumer: typically main loop
   returns true if got a sample
   ======================= */
static inline bool rb_pop(Ring *rb, Sample *out){
	uint32_t key = rb_enter_critical();
	if(rb->count == 0u){
		rb_exit_critical(key);
		return false;
	}

	*out = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1u) & RB_MASK;
    rb->count--;

	rb_exit_critical(key);
	return true;
}

static inline bool rb_peek_latest(Ring *rb, Sample *out){
  uint32_t key = rb_enter_critical();
  if(rb->count == 0u){
    rb_exit_critical(key);
    return false;
  }

  uint16_t latest_index = (rb->head + RB_CAPACITY - 1u) & RB_MASK;
  *out = rb->buf[latest_index];

  rb_exit_critical(key);
  return true;
}

static bool read_therm_and_ldr(float *therm_c, float *ldr_pct)
{
  uint16_t therm_adc = 0, ldr_adc = 0;

  if (HAL_ADC_Start(&hadc1) != HAL_OK) return false;

  if (adc_poll_get(&therm_adc) != HAL_OK) { HAL_ADC_Stop(&hadc1); return false; } // Rank1
  if (adc_poll_get(&ldr_adc)  != HAL_OK) { HAL_ADC_Stop(&hadc1); return false; } // Rank2

  HAL_ADC_Stop(&hadc1);

  if (therm_adc == 0 || therm_adc == 4095) return false;

  // --- thermistor convert (add clamp) ---
  float v = (therm_adc * 3.3f) / 4095.0f;
  if (v < 0.001f) v = 0.001f;
  if (v > 3.299f) v = 3.299f;

  float rth = R_FIXED * (3.3f / v - 1.0f);
  float t0 = 298.15f;
  float tempK = 1.0f / ((1.0f/t0) + (1.0f/BETA)*logf(rth / R0));
  *therm_c = tempK - 273.15f;

  *ldr_pct = ((float)ldr_adc / 4095.0f) * 100.0f;
  return true;
}

// Print one sample to UART (time, thermistor, photoresistor, optional BME280)
static void print_sample(const Sample *s)
{
    char line[256];
    int n = snprintf(line, sizeof(line),
                     "time_ms=%lu, thermistor=%.2f C, Photoresistor=%.2f%%",
                     (unsigned long)s->time_ms, s->thermistor_deg_c, s->ldr_pct);
    if (n < 0 || (size_t)n >= sizeof(line)) {
      uart_print("(sample line truncated)\r\n");
      return;
    }
    if (s->bme_ok) {
      snprintf(line + (size_t)n, sizeof(line) - (size_t)n,
               " | BME280: T=%.2f C P=%.2f hPa RH=%.2f%%\r\n",
               s->bme_temp_c, s->bme_press_hpa, s->bme_hum_pct);
    } else {
      snprintf(line + (size_t)n, sizeof(line) - (size_t)n, " | BME280: n/a\r\n");
    }
    uart_print(line);
}

void parse_command(char *cmd)
{
    /* Minimal UART command shell for runtime control and debugging. */

    if(strcmp(cmd, "start") == 0)
    {
        HAL_TIM_Base_Start_IT(&htim2);
        uart_print("Starting...\r\n");
    }
    else if(strcmp(cmd, "stop") == 0)
    {
        HAL_TIM_Base_Stop_IT(&htim2);
        uart_print("Stopping...\r\n");
    }
    else if (strcmp(cmd, "reset") == 0)
    {
        rb_init(&ringbuffer);
        sample_init(&sample);
        uart_print("Resetting...\r\n");
    }
    else if(strcmp(cmd, "status") == 0)
    {
        if (rb_peek_latest(&ringbuffer, &sample)) {
            print_sample(&sample);
        } else {
            uart_print("No data yet. Type 'start' and wait.\r\n");
        }
    }
    else if(strcmp(cmd, "help") == 0)
    {
        uart_print("Commands:\r\n");
        uart_print("  start  - Start sensor sampling\r\n");
        uart_print("  stop   - Stop sensor sampling\r\n");
        uart_print("  reset  - Clear ring buffer\r\n");
        uart_print("  status - Latest thermistor/LDR + BME280 (if OK)\r\n");
        uart_print("  help   - Show this message\r\n");
    }
    else
    {
    	uart_print("Unknown Command\r\n");
    }
}

void process_char(char c)
{
    /* Echo input and dispatch command when newline is received. */
    HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, 10);
    if(c == '\r' || c == '\n')
    {
        rx_buffer[rx_index] = '\0';
        parse_command(rx_buffer);
        rx_index = 0;
    }
    else
    {
        rx_buffer[rx_index++] = c;
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
	// last_blink: last time I toggled the LED (ms)
	// last_print: last time I printed to UART (ms)
	//uint32_t last_blink = 0;
	//uint32_t last_print = 0;
	//char line[128];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  rb_init(&ringbuffer);
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
  MX_IWDG_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  bme280_app_init();
  //HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart2, &rx_char, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* Timer ISR posts work via interrupt_flag; main loop consumes one sample each tick. */
    if (interrupt_flag) {
      interrupt_flag--;
      sample.time_ms = HAL_GetTick();
      if (read_therm_and_ldr(&sample.thermistor_deg_c, &sample.ldr_pct)) {
        read_bme280_into_sample(&sample);
        rb_push_overwrite(&ringbuffer, &sample);
      }
    }
    /* When persistent BME fault is latched, stop feeding IWDG to force safe reset. */
    if (!s_fault_latched) {
      HAL_IWDG_Refresh(&hiwdg);
    }
    /*
    while (rb_pop(&ringbuffer, &sample)) {
      snprintf(line, sizeof(line), "time_ms = %lu, thermistor = %.2f, Photoresistor = %.2f%%\r\n", sample.time_ms, sample.therm_deg_c, sample.ldr_pct);
      uart_print(line);
    }
    HAL_Delay(1);
    */
    //------------------
    /*
	  if(interrupt_flag){
		  interrupt_flag--;
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			    uint16_t therm_adc = 0;
			    uint16_t temp_adc  = 0;
			    uint16_t vref_adc  = 0;
			    uint16_t ldr_adc   = 0;


			    char msg[160];

			    // 1) Start ADC scan sequence (Rank1 -> Rank2 -> Rank3)
			    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
			      uart_print("ADC start error\r\n");
			    } else {
			      // 2) Read Rank 1 (thermistor)
			      if (adc_poll_get(&therm_adc) != HAL_OK) {
			        HAL_ADC_Stop(&hadc1);
			        uart_print("ADC timeout (rank1)\r\n");
			        continue; //start while loop again
			      }

			      if (adc_poll_get(&ldr_adc) != HAL_OK) {
			        HAL_ADC_Stop(&hadc1);
			        uart_print("ADC timeout (rank2)\r\n");
			        continue; //start while loop again
			      }

			      // 3) Read Rank 2 (internal temp sensor)
			      if (adc_poll_get(&temp_adc) != HAL_OK) {
			        HAL_ADC_Stop(&hadc1);
			        uart_print("ADC timeout (rank3)\r\n");
			        continue; //start while loop again
			      }

			      // 4) Read Rank 3 (vrefint)
			      if (adc_poll_get(&vref_adc) != HAL_OK) {
			        HAL_ADC_Stop(&hadc1);
			        uart_print("ADC timeout (rank4)\r\n");
			        continue; //start while loop again
			      }

			      // 5) Stop ADC (so it only runs when we ask)
			      HAL_ADC_Stop(&hadc1);

			      // ---- Calculate VDDA using Vrefint ----
			      // VDDA = (VrefInt * 4095) / vref_adc
			      float vdda = 3.3f;
			      if (vref_adc != 0) {
			        vdda = (VrefInt * 4095.0f) / (float)vref_adc;
			      }
			      if (therm_adc == 0 || therm_adc == 4095){
			    	  	HAL_ADC_Stop(&hadc1);
				        continue; //start while loop again
			      }

			      // ---- Convert internal temp sensor adc to temperature ----
			      // 1) therm_adc -> Vout
			      float vout = (therm_adc * vdda) / 4095.0f;

			      // 2) Vout -> Rth (케이스 A 예시)
			      float rth = R_FIXED * (vdda / vout - 1.0f);

			      // 3) Rth -> TempC (Beta)
			      float t0 = 298.15f;      // 25C in Kelvin
			      float tempK = 1.0f / ( (1.0f/t0) + (1.0f/BETA)*logf(rth / R0) );
			      float tempC = tempK - 273.15f;
			      // Vsense = temp_adc * VDDA / 4095
			      float vsense = ((float)temp_adc * vdda) / 4095.0f;

			      // TempC = 25 + (V25 - Vsense) / AvgSlope
			      float temp_c = (Voltage_25 - vsense) / AvgSlope + 25.0f;

			      // temp_x10 = temperature * 10 (example: 23.7°C -> 237)
			      int16_t temp_x10 = (int16_t)lroundf(temp_c * 10.0f);

			      // Print temp_x10 as X.Y
			      int16_t whole = temp_x10 / 10;
			      int16_t frac  = (temp_x10 < 0 ? -temp_x10 : temp_x10) % 10; //convert frac to absolute value to avoid printing - before the value

			      snprintf(msg, sizeof(msg),
			               "therm_adc=%u | temp_adc=%u | vref_adc=%u | VDDA=%.3fV | T=%d.%01dC | therm_C = %.2f | ldr_adc = %u\r\n",
			               (unsigned)therm_adc,
			               (unsigned)temp_adc,
			               (unsigned)vref_adc,
			               (double)vdda,
			               (int)whole, (int)frac, (float) tempC,
						   (unsigned)ldr_adc);

			      uart_print(msg);
			    }

	  }
          */
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 1499;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
	/* Keep ISR short: only signal main loop to do sensor work. */
	interrupt_flag++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* Byte-wise RX interrupt pipeline for UART command parser. */
        process_char(rx_char);
        HAL_UART_Receive_IT(&huart2, &rx_char, 1);
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
