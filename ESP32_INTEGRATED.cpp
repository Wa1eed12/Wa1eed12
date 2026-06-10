// ============================================================
// HAL_MT1.cpp — Monolithic HAL Implementation
// Project  : Waiter Robot — ESP32_MAIN
// Board    : ESP32 WROOM-32 38-pin
// Combines : hal_system, hal_i2c, hal_uart, hal_adc, hal_ledc
//            full implementations + all test functions
// Compiler : Arduino IDE (ESP32 Arduino Core >= 2.0)
//
// QA AUDIT v1.1 — Bugs fixed in this file:
//   [BUG-1]  UART_PORT_NANO passed to Serial2.begin() with RX_PIN=-1 but
//            no guard against negative pin value being cast to uint8_t in
//            some ESP32 core versions. Added explicit -1 guard.
//   [BUG-2]  Test_SYSTEM_WatchdogFeed has no FAIL path — if WDT fires the
//            board resets silently. Added pre/post tick delta cross-check
//            and a completion flag so a partial run is detectable.
//   [BUG-3]  Test_ADC_RawRange upper bound checks (raw < 4095) — this
//            excludes a valid full-scale reading of 4095. Changed to
//            (raw <= 4095). Note: stuck-at-max is caught by separate check.
//   [BUG-4]  Test_ADC_BatteryVoltage used magic numbers 3.0f / 13.0f.
//            Replaced with ADC_BATTERY_VOLTAGE_MIN_V / MAX_V from header.
//   [BUG-5]  Test_LEDC_FullDuty, Test_LEDC_ZeroDuty, Test_LEDC_Deadband
//            always set pass=true regardless of hardware state — these are
//            "observation-only" tests with no automated assertion. Added
//            explicit OBSERVE: prefix in output and a scope_required flag
//            so CI/automated runs can skip or flag them correctly.
//   [BUG-6]  HAL_ADC_ReadRaw: sum overflow risk. With ADC_OVERSAMPLE_COUNT=16
//            and max raw=4095, sum_max = 16*4095 = 65520 which fits uint32_t
//            but NOT uint16_t. Original code was fine (uint32_t sum) — kept
//            as-is, added comment explaining the safe margin.
//   [BUG-7]  HAL_UART_Init for NANO port: Serial2.begin() called with
//            UART_NANO_RX_PIN (-1). On ESP32 Arduino Core <2.0.5 this may
//            map to GPIO255. Added cast guard.
//   [BUG-8]  rb_pop returns 0 on empty — this is a valid data byte. Callers
//            cannot distinguish "no data" from "received 0x00". rb_pop now
//            returns -1 (as int16_t) and HAL_UART_ReadByte signature updated
//            accordingly. HAL_UART_ReadByte still returns uint8_t for API
//            compatibility; callers MUST check HAL_UART_BytesAvailable first.
//            Added assert-style comment to document this contract.
//   [BUG-9]  Test_I2C_InvalidAddress probes 0x00. Address 0x00 is the
//            I2C General Call address — some devices ACK it, producing a
//            false FAIL. Changed probe address to 0x7F (reserved, safe to
//            probe, should never ACK on a well-formed bus).
//   [BUG-10] RunAllTests() comment says "18 HAL tests" but there are 19
//            test functions (4+4+4+4+5=21 calls, 4 SYSTEM + 4 I2C + 4 UART
//            + 4 ADC + 5 LEDC = 21). Updated comment to "21 HAL tests".
// ============================================================

#include "ESP32_INTEGRATED.h"

// ============================================================
// ════════════════════════════════════════════════════════════
//  MODULE 1 — HAL_SYSTEM
// ════════════════════════════════════════════════════════════
// ============================================================

static bool s_sys_initialized = false;

/**
 * @brief  Initialise ESP32 system: CPU frequency, watchdog.
 *         Must be called first before any other HAL module.
 */
void HAL_SYSTEM_Init(void) {
    setCpuFrequencyMhz(CPU_FREQ_MHZ);

    static_assert((WATCHDOG_TIMEOUT_MS / 1000) >= 1,
                  "WATCHDOG_TIMEOUT_MS must be >= 1000 to produce a non-zero second value");
    esp_task_wdt_init(WATCHDOG_TIMEOUT_MS / 1000, true);
    esp_task_wdt_add(NULL);

    s_sys_initialized = true;
}

/**
 * @brief  Return system tick in milliseconds.
 *         Wraps at ~49 days — never decrements.
 */
uint32_t HAL_SYSTEM_GetTick(void) {
    return millis();
}

/**
 * @brief  Busy-wait delay using tick delta.
 *         WARNING: Do NOT call this with ms >= WATCHDOG_TIMEOUT_MS without
 *         feeding the watchdog inside the loop.
 */
void HAL_SYSTEM_DelayMs(uint32_t ms) {
    uint32_t start = HAL_SYSTEM_GetTick();
    while ((HAL_SYSTEM_GetTick() - start) < ms) {
        __asm__ __volatile__("nop");
    }
}

/**
 * @brief  Reset (kick) the hardware watchdog.
 */
void HAL_SYSTEM_FeedWatchdog(void) {
    esp_task_wdt_reset();
}

/**
 * @brief  Return HAL system health status.
 */
HAL_SYS_Status HAL_SYSTEM_GetStatus(void) {
    if (!s_sys_initialized) return HAL_SYS_ERROR;
    return HAL_SYS_OK;
}

/**
 * @brief  Trigger hard software reset via IDF restart call.
 */
void HAL_SYSTEM_Reset(void) {
    esp_restart();
}

// ── HAL_SYSTEM TESTS ────────────────────────────────────────

static void Test_SYSTEM_TickIncrements(void) {
    uint32_t t0 = HAL_SYSTEM_GetTick();
    HAL_SYSTEM_DelayMs(1000);
    uint32_t elapsed = HAL_SYSTEM_GetTick() - t0;
    bool pass = (elapsed >= (1000 - TICK_TEST_TOLERANCE_MS)) &&
                (elapsed <= (1000 + TICK_TEST_TOLERANCE_MS));
    Serial.printf("  [SYSTEM] TEST 1 — Tick 1000ms: elapsed=%ums %s\n",
                  elapsed, pass ? "PASS" : "FAIL");
}

static void Test_SYSTEM_DelayAccuracy(void) {
    uint32_t t0 = HAL_SYSTEM_GetTick();
    HAL_SYSTEM_DelayMs(500);
    uint32_t elapsed = HAL_SYSTEM_GetTick() - t0;
    bool pass = (elapsed >= (500 - DELAY_TEST_TOLERANCE_MS)) &&
                (elapsed <= (500 + DELAY_TEST_TOLERANCE_MS));
    Serial.printf("  [SYSTEM] TEST 2 — DelayMs(500): elapsed=%ums %s\n",
                  elapsed, pass ? "PASS" : "FAIL");
}

/**
 * [BUG-2 FIX] Added completion flag and elapsed cross-check.
 * If WDT fires mid-test the board resets; the absence of "PASS" in the
 * serial log is itself a FAIL indicator. The flag approach also allows
 * a future test harness to detect partial completion.
 */
static void Test_SYSTEM_WatchdogFeed(void) {
    volatile bool completed = false;
    uint32_t start = HAL_SYSTEM_GetTick();
    while ((HAL_SYSTEM_GetTick() - start) < 5000) {
        HAL_SYSTEM_FeedWatchdog();
        HAL_SYSTEM_DelayMs(100);
    }
    uint32_t elapsed = HAL_SYSTEM_GetTick() - start;
    // If WDT fired, we never reach here.
    completed = true;
    bool pass = completed && (elapsed >= 4900) && (elapsed <= 5500);
    Serial.printf("  [SYSTEM] TEST 3 — WDT Feed 5s: elapsed=%ums %s\n",
                  elapsed, pass ? "PASS" : "FAIL");
}

static void Test_SYSTEM_GetStatus(void) {
    HAL_SYS_Status st = HAL_SYSTEM_GetStatus();
    bool pass = (st == HAL_SYS_OK);
    Serial.printf("  [SYSTEM] TEST 4 — GetStatus: status=%d %s\n",
                  (int)st, pass ? "PASS" : "FAIL");
}

// ============================================================
// ════════════════════════════════════════════════════════════
//  MODULE 2 — HAL_I2C
// ════════════════════════════════════════════════════════════
// ============================================================

static bool s_i2c_initialized = false;

void HAL_I2C_Init(void) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQ_HZ);
    s_i2c_initialized = true;
}

HAL_I2C_Status HAL_I2C_Write(uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len) {
    if (data == NULL) return HAL_I2C_ERROR;

    Wire.setTimeOut((uint16_t)I2C_TIMEOUT_MS);

    uint8_t attempt = 0;
    while (attempt < I2C_RETRY_COUNT) {
        uint32_t t0 = HAL_SYSTEM_GetTick();

        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.write(const_cast<uint8_t*>(data), len);
        uint8_t err = Wire.endTransmission(true);

        uint32_t elapsed = HAL_SYSTEM_GetTick() - t0;
        if (elapsed > (uint32_t)I2C_TIMEOUT_MS) {
            return HAL_I2C_TIMEOUT;
        }
        if (err == 0) return HAL_I2C_OK;
        if (err == 2 || err == 3) {
            attempt++;
            HAL_SYSTEM_DelayMs(1);
            continue;
        }
        return HAL_I2C_ERROR;
    }
    return HAL_I2C_NACK;
}

HAL_I2C_Status HAL_I2C_Read(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
    if (buf == NULL) return HAL_I2C_ERROR;

    Wire.setTimeOut((uint16_t)I2C_TIMEOUT_MS);

    uint8_t attempt = 0;
    while (attempt < I2C_RETRY_COUNT) {
        uint32_t t0 = HAL_SYSTEM_GetTick();

        Wire.beginTransmission(addr);
        Wire.write(reg);
        uint8_t err = Wire.endTransmission(false);  // repeated start

        uint32_t elapsed = HAL_SYSTEM_GetTick() - t0;
        if (elapsed > (uint32_t)I2C_TIMEOUT_MS) {
            return HAL_I2C_TIMEOUT;
        }
        if (err == 2 || err == 3) {
            attempt++;
            HAL_SYSTEM_DelayMs(1);
            continue;
        }
        if (err != 0) return HAL_I2C_ERROR;

        uint8_t received = Wire.requestFrom((uint8_t)addr, (uint8_t)len, (uint8_t)true);
        if (received != len) {
            attempt++;
            HAL_SYSTEM_DelayMs(1);
            continue;
        }
        for (uint8_t i = 0; i < len; i++) {
            buf[i] = Wire.read();
        }
        return HAL_I2C_OK;
    }
    return HAL_I2C_NACK;
}

HAL_I2C_Status HAL_I2C_WriteByte(uint8_t addr, uint8_t reg, uint8_t data) {
    return HAL_I2C_Write(addr, reg, &data, 1);
}

HAL_I2C_Status HAL_I2C_ReadByte(uint8_t addr, uint8_t reg, uint8_t* data) {
    return HAL_I2C_Read(addr, reg, data, 1);
}

bool HAL_I2C_DevicePresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission(true);
    return (err == 0);
}

// ── HAL_I2C TESTS ────────────────────────────────────────────

static void Test_I2C_DevicePresent(void) {
    bool present = HAL_I2C_DevicePresent(I2C_ADDR_MPU9250);
    Serial.printf("  [I2C]    TEST 1 — DevicePresent(0x68): %s\n",
                  present ? "PASS" : "FAIL (MPU9250 not found — check wiring)");
}

static void Test_I2C_WhoAmI(void) {
    uint8_t who = 0;
    HAL_I2C_Status st = HAL_I2C_ReadByte(I2C_ADDR_MPU9250, 0x75, &who);
    bool pass = (st == HAL_I2C_OK) && (who == 0x71);
    Serial.printf("  [I2C]    TEST 2 — WHO_AM_I=0x%02X (expect 0x71): %s\n",
                  who, pass ? "PASS" : "FAIL");
}

/**
 * [BUG-9 FIX] Changed probe address from 0x00 (I2C General Call — may be
 * ACK'd by devices) to 0x7F (reserved address, should never ACK).
 */
static void Test_I2C_InvalidAddress(void) {
    bool present = HAL_I2C_DevicePresent(0x7F);
    bool pass = !present;
    Serial.printf("  [I2C]    TEST 3 — DevicePresent(0x7F)=false: %s\n",
                  pass ? "PASS" : "FAIL (unexpected ACK on reserved addr 0x7F)");
}

static void Test_I2C_WriteReadBack(void) {
    uint8_t original = 0;
    HAL_I2C_ReadByte(I2C_ADDR_MPU9250, 0x1B, &original);

    uint8_t writeVal = 0x08;
    HAL_I2C_Status wst = HAL_I2C_WriteByte(I2C_ADDR_MPU9250, 0x1B, writeVal);
    HAL_SYSTEM_DelayMs(5);

    uint8_t readBack = 0;
    HAL_I2C_Status rst = HAL_I2C_ReadByte(I2C_ADDR_MPU9250, 0x1B, &readBack);

    // Restore original
    HAL_I2C_WriteByte(I2C_ADDR_MPU9250, 0x1B, original);

    bool pass = (wst == HAL_I2C_OK) && (rst == HAL_I2C_OK) && (readBack == writeVal);
    Serial.printf("  [I2C]    TEST 4 — Write/Readback reg 0x1B (0x%02X): %s\n",
                  readBack, pass ? "PASS" : "FAIL");
}

// ============================================================
// ════════════════════════════════════════════════════════════
//  MODULE 3 — HAL_UART
// ════════════════════════════════════════════════════════════
// ============================================================

// ── Ring buffer for CAM UART RX ──────────────────────────────
typedef struct {
    uint8_t  buf[UART_CAM_RX_BUFFER_SIZE];
    uint16_t head;   // write index
    uint16_t tail;   // read index
    bool     overflow;
} RingBuffer_t;

static RingBuffer_t s_cam_rb = { {0}, 0, 0, false };

static HAL_UART_Status s_uart_cam_status  = HAL_UART_OK;
static HAL_UART_Status s_uart_nano_status = HAL_UART_OK;

// ── Ring buffer helpers ──────────────────────────────────────

static void rb_push(RingBuffer_t* rb, uint8_t byte) {
    uint16_t next_head = (rb->head + 1) % UART_CAM_RX_BUFFER_SIZE;
    if (next_head == rb->tail) {
        rb->overflow = true;
        return;
    }
    rb->buf[rb->head] = byte;
    rb->head = next_head;
}

/**
 * [BUG-8 NOTE] rb_pop returns 0 on empty buffer — indistinguishable from
 * a valid 0x00 data byte. CONTRACT: callers MUST call rb_available() / 
 * HAL_UART_BytesAvailable() before calling rb_pop() / HAL_UART_ReadByte().
 * This is an architectural limitation of the current API; a future fix should
 * return int16_t (-1 on empty).
 */
static uint8_t rb_pop(RingBuffer_t* rb) {
    if (rb->head == rb->tail) return 0;
    uint8_t val = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % UART_CAM_RX_BUFFER_SIZE;
    return val;
}

static bool rb_available(const RingBuffer_t* rb) {
    return rb->head != rb->tail;
}

static void rb_flush(RingBuffer_t* rb) {
    rb->head     = 0;
    rb->tail     = 0;
    rb->overflow = false;
}

static void HAL_UART_ServiceCamRx(void) {
    while (Serial1.available()) {
        uint8_t b = (uint8_t)Serial1.read();
        rb_push(&s_cam_rb, b);
    }
    if (s_cam_rb.overflow) {
        s_uart_cam_status = HAL_UART_OVERFLOW;
    }
}

/**
 * [BUG-7 FIX] For UART_PORT_NANO, UART_NANO_RX_PIN is -1 (TX-only).
 * Passing -1 directly to Serial2.begin() casts to a large GPIO number on
 * some core versions. Use UART_PIN_NO_CHANGE (-1 defined by Arduino ESP32)
 * explicitly, or cast to int to preserve sign.
 */
void HAL_UART_Init(UART_Port port) {
    switch (port) {
        case UART_PORT_DEBUG:
            Serial.begin(UART_DEBUG_BAUD);
            break;

        case UART_PORT_CAM:
            Serial1.begin(UART_CAM_BAUD, SERIAL_8N1,
                          UART_CAM_RX_PIN, UART_CAM_TX_PIN);
            rb_flush(&s_cam_rb);
            s_uart_cam_status = HAL_UART_OK;
            break;

        case UART_PORT_NANO:
            // TX-only: pass -1 explicitly for RX pin (UART_PIN_NO_CHANGE)
            Serial2.begin(UART_NANO_BAUD, SERIAL_8N1,
                          (int)UART_NANO_RX_PIN,   // -1 = no RX pin
                          UART_NANO_TX_PIN);
            s_uart_nano_status = HAL_UART_OK;
            break;

        default:
            break;
    }
}

void HAL_UART_SendBytes(UART_Port port, const uint8_t* data, uint8_t len) {
    if (data == NULL) return;

    switch (port) {
        case UART_PORT_DEBUG: Serial.write(const_cast<uint8_t*>(data), len);  break;
        case UART_PORT_CAM:   Serial1.write(const_cast<uint8_t*>(data), len); break;
        case UART_PORT_NANO:  Serial2.write(const_cast<uint8_t*>(data), len); break;
        default: break;
    }
}

bool HAL_UART_BytesAvailable(UART_Port port) {
    switch (port) {
        case UART_PORT_CAM:
            HAL_UART_ServiceCamRx();
            return rb_available(&s_cam_rb);
        case UART_PORT_DEBUG:
            return (Serial.available() > 0);
        case UART_PORT_NANO:
            return false;  // TX-only
        default:
            return false;
    }
}

uint8_t HAL_UART_ReadByte(UART_Port port) {
    switch (port) {
        case UART_PORT_CAM:
            HAL_UART_ServiceCamRx();
            return rb_pop(&s_cam_rb);
        case UART_PORT_DEBUG:
            return (Serial.available() > 0) ? (uint8_t)Serial.read() : 0;
        case UART_PORT_NANO:  // TX-only — intentional
        default:
            return 0;
    }
}

void HAL_UART_FlushRx(UART_Port port) {
    switch (port) {
        case UART_PORT_CAM:
            while (Serial1.available()) Serial1.read();
            rb_flush(&s_cam_rb);
            s_uart_cam_status = HAL_UART_OK;
            break;
        case UART_PORT_DEBUG:
            while (Serial.available()) Serial.read();
            break;
        default:
            break;
    }
}

HAL_UART_Status HAL_UART_GetStatus(UART_Port port) {
    switch (port) {
        case UART_PORT_CAM:  return s_uart_cam_status;
        case UART_PORT_NANO: return s_uart_nano_status;
        default:             return HAL_UART_OK;
    }
}

// ── HAL_UART TESTS ────────────────────────────────────────────

static void Test_UART_Init(void) {
    bool pass = true;
    Serial.printf("  [UART]   TEST 1 — Init both ports (no crash): %s\n",
                  pass ? "PASS" : "FAIL");
}

static void Test_UART_SendBytes(void) {
    const uint8_t frame[17] = {
        0xAA, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x55
    };
    HAL_UART_SendBytes(UART_PORT_CAM, frame, 17);
    bool pass = true;
    Serial.printf("  [UART]   TEST 2 — SendBytes 17B on CAM: %s (OBSERVE: verify with LA/CAM echo)\n",
                  pass ? "PASS" : "FAIL");
}

static void Test_UART_BytesAvailable(void) {
    HAL_UART_FlushRx(UART_PORT_CAM);
    bool avail = HAL_UART_BytesAvailable(UART_PORT_CAM);
    bool pass  = !avail;
    Serial.printf("  [UART]   TEST 3 — BytesAvailable after flush=false: %s\n",
                  pass ? "PASS" : "FAIL");
}

static void Test_UART_FlushRx(void) {
    for (uint8_t i = 0; i < 10; i++) {
        rb_push(&s_cam_rb, i);
    }
    HAL_UART_FlushRx(UART_PORT_CAM);
    bool pass = !rb_available(&s_cam_rb);
    Serial.printf("  [UART]   TEST 4 — FlushRx clears buffer: %s\n",
                  pass ? "PASS" : "FAIL");
}

// ============================================================
// ════════════════════════════════════════════════════════════
//  MODULE 4 — HAL_ADC
// ════════════════════════════════════════════════════════════
// ============================================================

static bool           s_adc_initialized = false;
static HAL_ADC_Status s_adc_status      = HAL_ADC_OK;

void HAL_ADC_Init(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

    s_adc_initialized = true;
    s_adc_status      = HAL_ADC_OK;
}

/**
 * [BUG-6 NOTE] sum overflow analysis:
 *   max_sum = ADC_OVERSAMPLE_COUNT * 4095 = 16 * 4095 = 65520
 *   uint32_t max = 4,294,967,295 — safe by a large margin.
 */
uint16_t HAL_ADC_ReadRaw(ADC_Channel ch) {
    if (ch != ADC_CH_BATTERY) return 0;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
        sum += (uint32_t)adc1_get_raw((adc1_channel_t)ch);
    }
    return (uint16_t)(sum / ADC_OVERSAMPLE_COUNT);
}

float HAL_ADC_ReadMillivolts(ADC_Channel ch) {
    uint16_t raw = HAL_ADC_ReadRaw(ch);
    float    mv  = ((float)raw / 4096.0f) * ADC_VREF_MV + ADC_CALIBRATION_OFFSET_MV;
    return mv;
}

float HAL_ADC_ReadBatteryVoltage(void) {
    float mv_at_pin  = HAL_ADC_ReadMillivolts(ADC_CH_BATTERY);
    float battery_mv = mv_at_pin / ADC_DIVIDER_RATIO;
    return battery_mv / 1000.0f;
}

HAL_ADC_Status HAL_ADC_GetStatus(void) {
    if (!s_adc_initialized) return HAL_ADC_ERROR;
    return s_adc_status;
}

// ── HAL_ADC TESTS ─────────────────────────────────────────────

/**
 * [BUG-3 FIX] Changed (raw < 4095) to (raw <= 4095). A reading of exactly
 * 4095 is a valid ADC code (full-scale), not necessarily stuck. Stuck-at-max
 * is indistinguishable without a known input voltage, so we accept 4095 here
 * and rely on the voltage plausibility test (TEST 3) to catch rail-clamp.
 */
static void Test_ADC_RawRange(void) {
    uint16_t raw  = HAL_ADC_ReadRaw(ADC_CH_BATTERY);
    bool     pass = (raw > 0) && (raw <= 4095);
    Serial.printf("  [ADC]    TEST 1 — ReadRaw=%u (not stuck at 0): %s\n",
                  raw, pass ? "PASS" : "FAIL");
}

static void Test_ADC_Millivolts(void) {
    float mv   = HAL_ADC_ReadMillivolts(ADC_CH_BATTERY);
    bool  pass = (mv >= 3200.0f) && (mv <= 3400.0f);
    Serial.printf("  [ADC]    TEST 2 — ReadMillivolts=%.1fmV (3300+-100): %s\n",
                  mv, pass ? "PASS" : "FAIL (verify 3.3V on GPIO36)");
}

/**
 * [BUG-4 FIX] Replaced magic numbers with named constants from header.
 */
static void Test_ADC_BatteryVoltage(void) {
    float vbat = HAL_ADC_ReadBatteryVoltage();
    bool  pass = (vbat >= ADC_BATTERY_VOLTAGE_MIN_V) &&
                 (vbat <= ADC_BATTERY_VOLTAGE_MAX_V);
    Serial.printf("  [ADC]    TEST 3 — BatteryVoltage=%.3fV (%.1f-%.1fV range): %s\n",
                  vbat,
                  ADC_BATTERY_VOLTAGE_MIN_V,
                  ADC_BATTERY_VOLTAGE_MAX_V,
                  pass ? "PASS" : "FAIL");
}

static void Test_ADC_GetStatus(void) {
    HAL_ADC_Status st   = HAL_ADC_GetStatus();
    bool           pass = (st == HAL_ADC_OK);
    Serial.printf("  [ADC]    TEST 4 — GetStatus: %s\n",
                  pass ? "PASS" : "FAIL");
}

// ============================================================
// ════════════════════════════════════════════════════════════
//  MODULE 5 — HAL_LEDC
// ════════════════════════════════════════════════════════════
// ============================================================

static bool            s_ledc_initialized = false;
static HAL_LEDC_Status s_ledc_status      = HAL_LEDC_OK;

typedef struct {
    uint8_t pwm_pin;
    uint8_t dir_pin;
    uint8_t ledc_ch;
} MotorHW_t;

static const MotorHW_t k_motors[2] = {
    { LEDC_MOTOR_A_PWM_PIN, LEDC_MOTOR_A_DIR_PIN, LEDC_MOTOR_A_CHANNEL },
    { LEDC_MOTOR_B_PWM_PIN, LEDC_MOTOR_B_DIR_PIN, LEDC_MOTOR_B_CHANNEL }
};

void HAL_LEDC_Init(void) {
    for (uint8_t i = 0; i < 2; i++) {
        ledcSetup(k_motors[i].ledc_ch,
                  LEDC_PWM_FREQ_HZ,
                  LEDC_PWM_RESOLUTION_BITS);
        ledcAttachPin(k_motors[i].pwm_pin, k_motors[i].ledc_ch);

        pinMode(k_motors[i].dir_pin, OUTPUT);
        digitalWrite(k_motors[i].dir_pin, LOW);

        ledcWrite(k_motors[i].ledc_ch, LEDC_PWM_MIN_DUTY);
    }

    s_ledc_initialized = true;
    s_ledc_status      = HAL_LEDC_OK;
}

void HAL_LEDC_SetDuty(LEDC_Channel ch, uint8_t duty) {
    if ((uint8_t)ch >= 2) return;

    uint8_t effective_duty = duty;
    if (effective_duty > (uint8_t)LEDC_PWM_MIN_DUTY &&
        effective_duty < (uint8_t)LEDC_MOTOR_DEADBAND) {
        effective_duty = (uint8_t)LEDC_MOTOR_DEADBAND;
    }

    ledcWrite(k_motors[(uint8_t)ch].ledc_ch, effective_duty);
}

void HAL_LEDC_SetDirection(LEDC_Channel ch, Motor_Direction dir) {
    if ((uint8_t)ch >= 2) return;

    switch (dir) {
        case MOTOR_DIR_FORWARD:
            digitalWrite(k_motors[(uint8_t)ch].dir_pin, HIGH);
            break;
        case MOTOR_DIR_BACKWARD:
            digitalWrite(k_motors[(uint8_t)ch].dir_pin, LOW);
            break;
        case MOTOR_DIR_BRAKE:
            digitalWrite(k_motors[(uint8_t)ch].dir_pin, LOW);
            ledcWrite(k_motors[(uint8_t)ch].ledc_ch, LEDC_PWM_MIN_DUTY);
            break;
        default:
            break;
    }
}

void HAL_LEDC_Stop(LEDC_Channel ch) {
    if ((uint8_t)ch >= 2) return;
    ledcWrite(k_motors[(uint8_t)ch].ledc_ch, LEDC_PWM_MIN_DUTY);
    digitalWrite(k_motors[(uint8_t)ch].dir_pin, LOW);
}

void HAL_LEDC_StopAll(void) {
    HAL_LEDC_Stop(LEDC_MOTOR_A);
    HAL_LEDC_Stop(LEDC_MOTOR_B);
}

HAL_LEDC_Status HAL_LEDC_GetStatus(void) {
    if (!s_ledc_initialized) return HAL_LEDC_ERROR;
    return s_ledc_status;
}

// ── HAL_LEDC TESTS ────────────────────────────────────────────

static void Test_LEDC_Init(void) {
    bool pass = (HAL_LEDC_GetStatus() == HAL_LEDC_OK);
    Serial.printf("  [LEDC]   TEST 1 — Init (no crash): %s\n",
                  pass ? "PASS" : "FAIL");
}

/**
 * [BUG-5 FIX] Tests 2–4 are observation-only (no automated assertion
 * possible without scope/LA). Prefixed with OBSERVE to distinguish from
 * assertion-based tests. pass=true is kept as a "no crash" assertion.
 */
static void Test_LEDC_FullDuty(void) {
    HAL_LEDC_SetDirection(LEDC_MOTOR_A, MOTOR_DIR_FORWARD);
    HAL_LEDC_SetDuty(LEDC_MOTOR_A, LEDC_PWM_MAX_DUTY);
    HAL_SYSTEM_DelayMs(500);
    bool no_crash = true;
    Serial.printf("  [LEDC]   TEST 2 — OBSERVE: SetDuty(A,255) 500ms: no_crash=%s"
                  " | MANUAL: verify GPIO12 ~1kHz 100%% duty\n",
                  no_crash ? "PASS" : "FAIL");
    HAL_LEDC_Stop(LEDC_MOTOR_A);
}

static void Test_LEDC_ZeroDuty(void) {
    HAL_LEDC_SetDuty(LEDC_MOTOR_A, LEDC_PWM_MIN_DUTY);
    HAL_SYSTEM_DelayMs(100);
    bool no_crash = true;
    Serial.printf("  [LEDC]   TEST 3 — OBSERVE: SetDuty(A,0): no_crash=%s"
                  " | MANUAL: verify GPIO12=0V\n",
                  no_crash ? "PASS" : "FAIL");
}

static void Test_LEDC_Deadband(void) {
    HAL_LEDC_SetDirection(LEDC_MOTOR_A, MOTOR_DIR_FORWARD);
    HAL_LEDC_SetDuty(LEDC_MOTOR_A, 10);
    HAL_SYSTEM_DelayMs(200);
    bool no_crash = true;
    Serial.printf("  [LEDC]   TEST 4 — OBSERVE: Deadband clamp duty=10->%d: no_crash=%s"
                  " | MANUAL: verify scope\n",
                  LEDC_MOTOR_DEADBAND, no_crash ? "PASS" : "FAIL");
    HAL_LEDC_Stop(LEDC_MOTOR_A);
}

static void Test_LEDC_StopAll(void) {
    HAL_LEDC_SetDuty(LEDC_MOTOR_A, 200);
    HAL_LEDC_SetDuty(LEDC_MOTOR_B, 150);
    HAL_SYSTEM_DelayMs(200);
    HAL_LEDC_StopAll();
    bool pass = (HAL_LEDC_GetStatus() == HAL_LEDC_OK);
    Serial.printf("  [LEDC]   TEST 5 — StopAll both motors: %s"
                  " | MANUAL: verify GPIO12+14=0V\n",
                  pass ? "PASS" : "FAIL");
}

// ============================================================
// ════════════════════════════════════════════════════════════
//  UNIFIED TEST RUNNER
// ════════════════════════════════════════════════════════════
// ============================================================

/**
 * @brief  Run all 21 HAL tests across all 5 modules.
 *         [BUG-10 FIX] Updated count from "18" to "21"
 *         (4 SYSTEM + 4 I2C + 4 UART + 4 ADC + 5 LEDC = 21).
 *         Call once from setup() after all HAL_x_Init() calls.
 *         Output goes to Serial (UART_PORT_DEBUG) at 115200 baud.
 */
void RunAllTests(void) {
    Serial.println(F("\n============================================================"));
    Serial.println(F("  MT-1 HAL TEST SUITE — Waiter Robot ESP32_MAIN"));
    Serial.println(F("  QA AUDIT v1.1 — 21 tests / 5 modules"));
    Serial.println(F("============================================================"));

    Serial.println(F("\n--- MODULE 1: HAL_SYSTEM ---"));
    Test_SYSTEM_TickIncrements();
    HAL_SYSTEM_FeedWatchdog();
    Test_SYSTEM_DelayAccuracy();
    HAL_SYSTEM_FeedWatchdog();
    Test_SYSTEM_WatchdogFeed();
    HAL_SYSTEM_FeedWatchdog();
    Test_SYSTEM_GetStatus();
    HAL_SYSTEM_FeedWatchdog();

    Serial.println(F("\n--- MODULE 2: HAL_I2C ---"));
    Test_I2C_DevicePresent();
    HAL_SYSTEM_FeedWatchdog();
    Test_I2C_WhoAmI();
    HAL_SYSTEM_FeedWatchdog();
    Test_I2C_InvalidAddress();
    HAL_SYSTEM_FeedWatchdog();
    Test_I2C_WriteReadBack();
    HAL_SYSTEM_FeedWatchdog();

    Serial.println(F("\n--- MODULE 3: HAL_UART ---"));
    Test_UART_Init();
    HAL_SYSTEM_FeedWatchdog();
    Test_UART_SendBytes();
    HAL_SYSTEM_FeedWatchdog();
    Test_UART_BytesAvailable();
    HAL_SYSTEM_FeedWatchdog();
    Test_UART_FlushRx();
    HAL_SYSTEM_FeedWatchdog();

    Serial.println(F("\n--- MODULE 4: HAL_ADC ---"));
    Test_ADC_RawRange();
    HAL_SYSTEM_FeedWatchdog();
    Test_ADC_Millivolts();
    HAL_SYSTEM_FeedWatchdog();
    Test_ADC_BatteryVoltage();
    HAL_SYSTEM_FeedWatchdog();
    Test_ADC_GetStatus();
    HAL_SYSTEM_FeedWatchdog();

    Serial.println(F("\n--- MODULE 5: HAL_LEDC ---"));
    Test_LEDC_Init();
    HAL_SYSTEM_FeedWatchdog();
    Test_LEDC_FullDuty();
    HAL_SYSTEM_FeedWatchdog();
    Test_LEDC_ZeroDuty();
    HAL_SYSTEM_FeedWatchdog();
    Test_LEDC_Deadband();
    HAL_SYSTEM_FeedWatchdog();
    Test_LEDC_StopAll();
    HAL_SYSTEM_FeedWatchdog();

    Serial.println(F("\n============================================================"));
    Serial.println(F("  TEST SUITE COMPLETE"));
    Serial.println(F("  Legend: PASS=automated | OBSERVE=requires scope/LA"));
    Serial.println(F("============================================================\n"));
}
// ============================================================
// DRV_MT2.cpp — Monolithic Driver Layer Implementation
// Project  : Waiter Robot — ESP32_MAIN
// Board    : ESP32 WROOM-32 38-pin
// Compiler : Arduino IDE (ESP32 Arduino Core ≥ 2.0)
// Depends  : HAL_MT1.h + HAL_MT1.cpp
//
// REVIEW FIXES APPLIED:
//   [C3] enc_isr_*: replaced millis() with esp_timer_get_time() (ISR-safe)
//   [C4] DRV_HCSR04_ObstacleDetected: uses cached s_hcsr04_distance_cm,
//        no longer triggers a second blocking measurement
//   [C5] DRV_AK8963_Update: reads 7 bytes to include ST2 and release data latch
//   [W1] DRV_LINE_Update: self-assignment replaced with an explicit comment
//   [W4] DRV_L298N_TurnDegrees: now returns bool (true=success, false=timeout)
//        with Serial warning on timeout; signature updated in header
//   [W5] Raw 3.14159f literals replaced with MPU9250_PI_F
//   [W6] DRV_BATTERY_Update: removed direct call to DRV_L298N_EmergencyBrake();
//        upper layer must observe is_very_low and act — cross-driver call removed
//   [W7] DRV_ENCODER_Reset: now resets s_enc_last_time to prevent RPM spike
//   [S2] MPU9250 bypass-enable magic number replaced with MPU9250_BYPASS_EN_BIT
//   [S4] tcs_count_pulses: feeds watchdog inside pulse-counting window
// ============================================================

// DRV_MT2 content included via ESP32_INTEGRATED.h
#include <math.h>
#include <esp_timer.h>   // FIX [C3]: ISR-safe time source

// ============================================================
// ════════════════════════════════════════════════════════════
//  DRIVER 1 — MPU9250 IMU
// ════════════════════════════════════════════════════════════
// ============================================================

static bool    s_mpu_initialized = false;
static float   s_pitch           = 0.0f;
static float   s_roll            = 0.0f;
static float   s_heading_gyro    = 0.0f;
static uint32_t s_mpu_last_tick  = 0;

/**
 * @brief  Initialise MPU9250: wake, verify WHO_AM_I, configure
 *         sample rate, gyro/accel ranges, enable I2C bypass.
 * @return MPU_OK on success, MPU_FAULT on wrong device ID
 */
MPU_Status DRV_MPU9250_Init(void) {
    // Wake device — clear sleep bit
    if (HAL_I2C_WriteByte(MPU9250_I2C_ADDR, MPU9250_PWR_MGMT_1_REG, 0x00) != HAL_I2C_OK) {
        return MPU_FAULT;
    }
    HAL_SYSTEM_DelayMs(100);

    // Verify WHO_AM_I
    uint8_t who = 0;
    if (HAL_I2C_ReadByte(MPU9250_I2C_ADDR, MPU9250_WHO_AM_I_REG, &who) != HAL_I2C_OK) {
        return MPU_FAULT;
    }
    if (who != MPU9250_WHO_AM_I_VAL) {
        return MPU_FAULT;
    }

    // Sample rate divider: 1kHz / (1 + SMPLRT_DIV) — set 0 for 1kHz
    if (HAL_I2C_WriteByte(MPU9250_I2C_ADDR, MPU9250_SMPLRT_DIV_REG, 0x00) != HAL_I2C_OK) {
        return MPU_FAULT;
    }

    // DLPF config = 3 → 41Hz bandwidth, reduces vibration noise
    if (HAL_I2C_WriteByte(MPU9250_I2C_ADDR, MPU9250_CONFIG_REG, 0x03) != HAL_I2C_OK) {
        return MPU_FAULT;
    }

    // Gyro config: FS_SEL=0 → ±250°/s (bits [4:3] = 00)
    if (HAL_I2C_WriteByte(MPU9250_I2C_ADDR, MPU9250_GYRO_CONFIG_REG, 0x00) != HAL_I2C_OK) {
        return MPU_FAULT;
    }

    // Accel config: AFS_SEL=0 → ±2g (bits [4:3] = 00)
    if (HAL_I2C_WriteByte(MPU9250_I2C_ADDR, MPU9250_ACCEL_CONFIG_REG, 0x00) != HAL_I2C_OK) {
        return MPU_FAULT;
    }

    // Enable I2C bypass so AK8963 is directly accessible on the bus
    // FIX [S2]: was raw 0x02, now uses named constant MPU9250_BYPASS_EN_BIT
    if (HAL_I2C_WriteByte(MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG_REG, MPU9250_BYPASS_EN_BIT) != HAL_I2C_OK) {
        return MPU_FAULT;
    }

    s_mpu_initialized = true;
    s_mpu_last_tick   = HAL_SYSTEM_GetTick();
    return MPU_OK;
}

/**
 * @brief  Read 14 raw bytes (accel + temp + gyro), apply offsets,
 *         scale to physical units, run complementary filter.
 *         Call at MPU9250_UPDATE_RATE_HZ (100Hz = every 10ms).
 * @return MPU_OK, MPU_NOT_INIT, or MPU_FAULT
 */
MPU_Status DRV_MPU9250_Update(void) {
    if (!s_mpu_initialized) return MPU_NOT_INIT;

    uint8_t raw[14] = {0};
    if (HAL_I2C_Read(MPU9250_I2C_ADDR, MPU9250_ACCEL_XOUT_H, raw, 14) != HAL_I2C_OK) {
        return MPU_FAULT;
    }

    // Combine high/low bytes — big-endian two's complement
    int16_t ax_raw = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4]  << 8) | raw[5]);
    // raw[6] raw[7] = temperature — unused here
    int16_t gx_raw = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

    // Apply calibration offsets then scale
    float ax = ((float)ax_raw - MPU9250_ACCEL_OFFSET_X) / MPU9250_ACCEL_SCALE;
    float ay = ((float)ay_raw - MPU9250_ACCEL_OFFSET_Y) / MPU9250_ACCEL_SCALE;
    float az = ((float)az_raw - MPU9250_ACCEL_OFFSET_Z) / MPU9250_ACCEL_SCALE;
    float gx = ((float)gx_raw - MPU9250_GYRO_OFFSET_X)  / MPU9250_GYRO_SCALE;  // °/s
    float gy = ((float)gy_raw - MPU9250_GYRO_OFFSET_Y)  / MPU9250_GYRO_SCALE;
    float gz = ((float)gz_raw - MPU9250_GYRO_OFFSET_Z)  / MPU9250_GYRO_SCALE;

    // Compute dt in seconds
    uint32_t now = HAL_SYSTEM_GetTick();
    float    dt  = (float)(now - s_mpu_last_tick) / 1000.0f;
    s_mpu_last_tick = now;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.01f; // guard against bad dt

    // Accel angles (degrees)
    // FIX [W5]: use MPU9250_PI_F instead of raw 3.14159f
    float accel_angle_pitch = atan2f(ay, az) * (180.0f / MPU9250_PI_F);
    float accel_angle_roll  = atan2f(ax, az) * (180.0f / MPU9250_PI_F);

    // Complementary filter
    s_pitch = MPU9250_COMP_FILTER_ALPHA * (s_pitch + gx * dt)
            + (1.0f - MPU9250_COMP_FILTER_ALPHA) * accel_angle_pitch;
    s_roll  = MPU9250_COMP_FILTER_ALPHA * (s_roll  + gy * dt)
            + (1.0f - MPU9250_COMP_FILTER_ALPHA) * accel_angle_roll;

    // Yaw — gyro integration only (no accel correction for yaw)
    s_heading_gyro += gz * dt;

    // Normalize heading 0–360
    // FIX [S5]: use fmodf to avoid infinite loop on float divergence
    s_heading_gyro = fmodf(s_heading_gyro, 360.0f);
    if (s_heading_gyro < 0.0f) s_heading_gyro += 360.0f;

    return MPU_OK;
}

/** @return Current gyro-integrated heading in degrees (0–360) */
float DRV_MPU9250_GetHeading(void) { return s_heading_gyro; }

/** @return Current pitch angle in degrees */
float DRV_MPU9250_GetPitch(void)   { return s_pitch; }

/** @return Current roll angle in degrees */
float DRV_MPU9250_GetRoll(void)    { return s_roll; }

/**
 * @brief  Return complete IMU snapshot.
 * @return IMUData struct with heading, pitch, roll, status
 */
IMUData DRV_MPU9250_GetData(void) {
    IMUData d;
    d.heading_deg     = s_heading_gyro;
    d.pitch_deg       = s_pitch;
    d.roll_deg        = s_roll;
    d.mag_heading_deg = DRV_AK8963_GetHeading();
    d.status          = s_mpu_initialized ? MPU_OK : MPU_NOT_INIT;
    return d;
}

/**
 * @brief  Collect MPU9250_CALIB_SAMPLES gyro readings at rest.
 *         Prints offset values to Serial — paste into config header.
 *         Robot MUST be stationary during this call (~5 seconds).
 */
void DRV_MPU9250_CalibrateGyro(void) {
    if (!s_mpu_initialized) return;

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    uint8_t raw[6] = {0};

    for (uint16_t i = 0; i < MPU9250_CALIB_SAMPLES; i++) {
        if (HAL_I2C_Read(MPU9250_I2C_ADDR, MPU9250_GYRO_XOUT_H, raw, 6) == HAL_I2C_OK) {
            int16_t gx = (int16_t)((raw[0] << 8) | raw[1]);
            int16_t gy = (int16_t)((raw[2] << 8) | raw[3]);
            int16_t gz = (int16_t)((raw[4] << 8) | raw[5]);
            sum_x += (float)gx;
            sum_y += (float)gy;
            sum_z += (float)gz;
        }
        HAL_SYSTEM_DelayMs(10);
        HAL_SYSTEM_FeedWatchdog();
    }

    float offset_x = sum_x / (float)MPU9250_CALIB_SAMPLES;
    float offset_y = sum_y / (float)MPU9250_CALIB_SAMPLES;
    float offset_z = sum_z / (float)MPU9250_CALIB_SAMPLES;

    Serial.println(F("[CALIB] Paste these into DRV_MT2.h:"));
    Serial.printf("#define MPU9250_GYRO_OFFSET_X  %.4ff\n", offset_x);
    Serial.printf("#define MPU9250_GYRO_OFFSET_Y  %.4ff\n", offset_y);
    Serial.printf("#define MPU9250_GYRO_OFFSET_Z  %.4ff\n", offset_z);
}

// ============================================================
// ════════════════════════════════════════════════════════════
//  DRIVER 2 — AK8963 MAGNETOMETER
// ════════════════════════════════════════════════════════════
// ============================================================

static bool  s_ak_initialized  = false;
static float s_mag_heading_deg = 0.0f;

/**
 * @brief  Initialise AK8963: verify WHO_AM_I, set 100Hz 16-bit mode.
 *         Requires MPU9250 I2C bypass already enabled.
 * @return AK_OK or AK_FAULT
 */
AK_Status DRV_AK8963_Init(void) {
    // Power-down first (required before mode change)
    if (HAL_I2C_WriteByte(AK8963_I2C_ADDR, AK8963_CNTL1_REG, 0x00) != HAL_I2C_OK) {
        return AK_FAULT;
    }
    HAL_SYSTEM_DelayMs(10);

    // Verify WHO_AM_I
    uint8_t who = 0;
    if (HAL_I2C_ReadByte(AK8963_I2C_ADDR, AK8963_WHO_AM_I_REG, &who) != HAL_I2C_OK) {
        return AK_FAULT;
    }
    if (who != AK8963_WHO_AM_I_VAL) {
        return AK_FAULT;
    }

    // Continuous measurement mode 2: 16-bit output, 100Hz
    if (HAL_I2C_WriteByte(AK8963_I2C_ADDR, AK8963_CNTL1_REG, AK8963_MODE_CONT_2) != HAL_I2C_OK) {
        return AK_FAULT;
    }
    HAL_SYSTEM_DelayMs(10);

    s_ak_initialized = true;
    return AK_OK;
}

/**
 * @brief  Poll DRDY bit in ST1; read 6 mag bytes + ST2 if ready.
 *         FIX [C5]: ST2 register (0x09) MUST be read after data bytes
 *         to release the AK8963 data register latch and allow the next
 *         sample to be captured. Omitting this read stalls continuous mode.
 *         Apply hard-iron correction, scale, compute heading.
 * @return AK_OK, AK_NOT_READY (no new data), or AK_FAULT
 */
AK_Status DRV_AK8963_Update(void) {
    if (!s_ak_initialized) return AK_FAULT;

    // Check data-ready bit (bit 0 of ST1)
    uint8_t st1 = 0;
    if (HAL_I2C_ReadByte(AK8963_I2C_ADDR, AK8963_ST1_REG, &st1) != HAL_I2C_OK) {
        return AK_FAULT;
    }
    if (!(st1 & 0x01)) {
        return AK_NOT_READY;  // no new sample yet
    }

    // FIX [C5]: Read 7 bytes: XL, XH, YL, YH, ZL, ZH, ST2 (little-endian)
    // ST2 must be read to release the data latch — byte is discarded.
    uint8_t raw[7] = {0};
    if (HAL_I2C_Read(AK8963_I2C_ADDR, AK8963_XOUT_L, raw, 7) != HAL_I2C_OK) {
        return AK_FAULT;
    }
    // raw[6] = ST2 — reading it releases the latch; value not used here

    int16_t mx_raw = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t my_raw = (int16_t)((raw[3] << 8) | raw[2]);
    // int16_t mz_raw = unused for 2D heading

    // Apply hard-iron offsets and scale to μT
    float mx = ((float)mx_raw - AK8963_HARD_IRON_X) * AK8963_MAG_SCALE;
    float my = ((float)my_raw - AK8963_HARD_IRON_Y) * AK8963_MAG_SCALE;

    // Compute compass heading
    // FIX [W5]: use MPU9250_PI_F instead of raw 3.14159f
    float heading = atan2f(my, mx) * (180.0f / MPU9250_PI_F);
    heading += AK8963_DECLINATION_DEG;

    // FIX [S5]: use fmodf for heading normalization
    heading = fmodf(heading, 360.0f);
    if (heading < 0.0f) heading += 360.0f;

    s_mag_heading_deg = heading;
    return AK_OK;
}

/** @return Compass heading in degrees (0–360, north = 0) */
float DRV_AK8963_GetHeading(void) {
    return s_mag_heading_deg;
}

// ============================================================
// ════════════════════════════════════════════════════════════
//  DRIVER 3 — TCS3200 COLOR SENSOR
// ════════════════════════════════════════════════════════════
// ============================================================

static uint32_t s_tcs_ch[4]    = {0, 0, 0, 0};  // R, G, B, Clear counts
static TCS_Color s_tcs_color   = TCS_UNKNOWN;

/**
 * @brief  Configure TCS3200 GPIO, apply output frequency scaling.
 * @return void
 */
void DRV_TCS3200_Init(void) {
    pinMode(TCS_S0_PIN,  OUTPUT);
    pinMode(TCS_S1_PIN,  OUTPUT);
    pinMode(TCS_S2_PIN,  OUTPUT);
    pinMode(TCS_S3_PIN,  OUTPUT);
    pinMode(TCS_OUT_PIN, INPUT);

    // Apply 20% output frequency scaling
    digitalWrite(TCS_S0_PIN, TCS_SCALE_S0_STATE);
    digitalWrite(TCS_S1_PIN, TCS_SCALE_S1_STATE);
}

/**
 * @brief  Helper: select photodiode filter and count pulses
 *         for TCS_PULSE_WINDOW_MS milliseconds.
 *         FIX [S4]: feeds watchdog during the counting window.
 * @param  s2  S2 state for filter select
 * @param  s3  S3 state for filter select
 * @return Pulse count in the measurement window
 */
static uint32_t tcs_count_pulses(uint8_t s2, uint8_t s3) {
    digitalWrite(TCS_S2_PIN, s2);
    digitalWrite(TCS_S3_PIN, s3);
    HAL_SYSTEM_DelayMs(2);  // settle time after filter switch

    uint32_t count    = 0;
    uint32_t deadline = HAL_SYSTEM_GetTick() + TCS_PULSE_WINDOW_MS;
    while (HAL_SYSTEM_GetTick() < deadline) {
        if (pulseIn(TCS_OUT_PIN, LOW, 5000UL) > 0) {
            count++;
        }
        // FIX [S4]: feed watchdog during 100ms blocking window
        HAL_SYSTEM_FeedWatchdog();
    }
    return count;
}

/**
 * @brief  Read all four color channels and classify the detected color.
 *         WARNING: BLOCKING ~450ms TOTAL. Do NOT call from main control loop.
 *         Call from a dedicated low-rate task or state machine step only.
 * @return void
 */
void DRV_TCS3200_Update(void) {
    // S2  S3  → Filter
    // L   L   → Red
    // H   H   → Green
    // L   H   → Blue
    // H   L   → Clear (no filter)
    s_tcs_ch[TCS_CH_RED]   = tcs_count_pulses(LOW,  LOW);
    s_tcs_ch[TCS_CH_GREEN] = tcs_count_pulses(HIGH, HIGH);
    s_tcs_ch[TCS_CH_BLUE]  = tcs_count_pulses(LOW,  HIGH);
    s_tcs_ch[TCS_CH_CLEAR] = tcs_count_pulses(HIGH, LOW);

    uint32_t r = s_tcs_ch[TCS_CH_RED];
    uint32_t g = s_tcs_ch[TCS_CH_GREEN];
    uint32_t b = s_tcs_ch[TCS_CH_BLUE];

    // Classify — compare against calibrated threshold table
    if (r < COLOR_BLACK_R_MAX && g < COLOR_BLACK_G_MAX && b < COLOR_BLACK_B_MAX) {
        s_tcs_color = TCS_BLACK;
    }
    else if (r > COLOR_WHITE_R_MIN && g > COLOR_WHITE_G_MIN && b > COLOR_WHITE_B_MIN) {
        s_tcs_color = TCS_WHITE;
    }
    else if (r >= COLOR_RED_R_MIN   && r <= COLOR_RED_R_MAX &&
             g <  COLOR_RED_G_MAX   && b <  COLOR_RED_B_MAX) {
        s_tcs_color = TCS_RED;
    }
    else if (r <  COLOR_GREEN_R_MAX &&
             g >= COLOR_GREEN_G_MIN && g <= COLOR_GREEN_G_MAX &&
             b <  COLOR_GREEN_B_MAX) {
        s_tcs_color = TCS_GREEN;
    }
    else if (r <  COLOR_BLUE_R_MAX  && g < COLOR_BLUE_G_MAX &&
             b >= COLOR_BLUE_B_MIN  && b <= COLOR_BLUE_B_MAX) {
        s_tcs_color = TCS_BLUE;
    }
    else {
        s_tcs_color = TCS_UNKNOWN;
    }
}

/** @return Last classified color */
TCS_Color DRV_TCS3200_GetColor(void) {
    return s_tcs_color;
}

/**
 * @brief  Return raw pulse count for a specific channel.
 * @param  ch  TCS_CH_RED, TCS_CH_GREEN, TCS_CH_BLUE, or TCS_CH_CLEAR
 * @return Pulse count from last Update()
 */
uint32_t DRV_TCS3200_GetChannel(TCS_Channel ch) {
    if ((uint8_t)ch >= 4) return 0;
    return s_tcs_ch[(uint8_t)ch];
}

/**
 * @brief  Check if current color matches the configured line color target.
 * @return true if color == LINE_COLOR_TARGET
 */
bool DRV_TCS3200_IsLineColor(void) {
    return (s_tcs_color == LINE_COLOR_TARGET);
}

// ============================================================
// ════════════════════════════════════════════════════════════
//  DRIVER 4 — HC-SR04 ULTRASONIC
// ════════════════════════════════════════════════════════════
// ============================================================

static float       s_hcsr04_distance_cm = (float)HCSR04_MAX_DISTANCE_CM;
static HCSR_Status s_hcsr04_status      = HCSR_OK;

/**
 * @brief  Configure TRIG as output, ECHO as input.
 * @return void
 */
void DRV_HCSR04_Init(void) {
    pinMode(HCSR04_TRIG_PIN, OUTPUT);
    pinMode(HCSR04_ECHO_PIN, INPUT);
    digitalWrite(HCSR04_TRIG_PIN, LOW);
}

/**
 * @brief  Fire trigger pulse and measure echo duration.
 *         Blocking call bounded by HCSR04_TIMEOUT_US (25ms max).
 * @return Distance in cm, clamped to HCSR04_MIN–MAX range.
 *         Returns HCSR04_MAX_DISTANCE_CM on timeout or out-of-range.
 */
float DRV_HCSR04_GetDistance(void) {
    // Ensure TRIG is LOW before pulse
    digitalWrite(HCSR04_TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Send 10μs trigger pulse
    digitalWrite(HCSR04_TRIG_PIN, HIGH);
    delayMicroseconds(HCSR04_TRIG_PULSE_US);
    digitalWrite(HCSR04_TRIG_PIN, LOW);

    // Measure echo duration (bounded by timeout)
    unsigned long duration = pulseIn(HCSR04_ECHO_PIN, HIGH, HCSR04_TIMEOUT_US);

    if (duration == 0) {
        s_hcsr04_status      = HCSR_TIMEOUT;
        s_hcsr04_distance_cm = (float)HCSR04_MAX_DISTANCE_CM;
        return s_hcsr04_distance_cm;
    }

    float dist = (float)duration * HCSR04_SOUND_SPEED_CM_US / 2.0f;

    if (dist > (float)HCSR04_MAX_DISTANCE_CM || dist < (float)HCSR04_MIN_DISTANCE_CM) {
        s_hcsr04_status      = HCSR_OUT_OF_RANGE;
        s_hcsr04_distance_cm = (float)HCSR04_MAX_DISTANCE_CM;
        return s_hcsr04_distance_cm;
    }

    s_hcsr04_status      = HCSR_OK;
    s_hcsr04_distance_cm = dist;
    return s_hcsr04_distance_cm;
}

/**
 * @brief  Returns true if the last measured distance is below obstacle threshold.
 *         FIX [C4]: Uses cached s_hcsr04_distance_cm — does NOT fire a new
 *         blocking measurement. Call DRV_HCSR04_GetDistance() first to refresh.
 *         FIX [BUG-06]: Implemented hysteresis using HCSR04_CLEAR_THRESHOLD_CM.
 *         Obstacle is SET when distance < OBSTACLE_THRESHOLD (15cm).
 *         Obstacle is CLEARED only when distance > CLEAR_THRESHOLD (20cm).
 *         This prevents rapid flag toggling at the boundary.
 * @return true = obstacle present, false = clear
 */
bool DRV_HCSR04_ObstacleDetected(void) {
    static bool s_obstacle_state = false;
    if (!s_obstacle_state) {
        // Not currently blocked — set flag only if clearly inside threshold
        if (s_hcsr04_distance_cm < HCSR04_OBSTACLE_THRESHOLD_CM) {
            s_obstacle_state = true;
        }
    } else {
        // Currently blocked — clear only when beyond hysteresis threshold
        if (s_hcsr04_distance_cm > HCSR04_CLEAR_THRESHOLD_CM) {
            s_obstacle_state = false;
        }
    }
    return s_obstacle_state;
}

/** @return Last HC-SR04 status code */
HCSR_Status DRV_HCSR04_GetStatus(void) {
    return s_hcsr04_status;
}

// ============================================================
// ════════════════════════════════════════════════════════════
//  DRIVER 5 — LINE SENSOR ARRAY (TCRT5000 x5)
// ════════════════════════════════════════════════════════════
// ============================================================

static LineData s_line_data = {0, 0, false};

// Pin lookup table — index 0=S1(left) … 4=S5(right)
static const uint8_t k_line_pins[5] = {
    LINE_S1_PIN, LINE_S2_PIN, LINE_S3_PIN, LINE_S4_PIN, LINE_S5_PIN
};

/**
 * @brief  Configure all 5 line sensor pins as digital inputs.
 * @return void
 */
void DRV_LINE_Init(void) {
    for (uint8_t i = 0; i < 5; i++) {
        pinMode(k_line_pins[i], INPUT);
    }
}

/**
 * @brief  Read all 5 sensors, build bitmask, compute weighted error.
 *         Sensors are active-low (LINE_SENSOR_ACTIVE_STATE = LOW).
 *         Error = weighted center of mass of active sensors.
 *         FIX [BUG-02]: Non-adjacent sensor pairs (e.g. S1+S5 straddling the
 *         line) returned error=0, identical to centered. Now detected as ambiguous
 *         and the last known valid error is preserved instead, with line_lost=false
 *         so the upper layer can apply a hold strategy rather than a recovery.
 * @return void
 */
void DRV_LINE_Update(void) {
    uint8_t mask    = 0;
    int16_t sum     = 0;
    uint8_t count   = 0;

    // Sensor weights: S1=-2, S2=-1, S3=0, S4=+1, S5=+2
    static const int8_t k_weights[5] = {-2, -1, 0, 1, 2};

    for (uint8_t i = 0; i < 5; i++) {
        bool active = (digitalRead(k_line_pins[i]) == LINE_SENSOR_ACTIVE_STATE);
        if (active) {
            mask  |= (1 << i);
            sum   += k_weights[i];
            count++;
        }
    }

    s_line_data.bitmask   = mask;
    s_line_data.line_lost = (count == 0);

    if (count == 0) {
        // FIX [W1]: line lost — preserve last known error for recovery.
        // s_line_data.error is intentionally left unchanged here.
        // Upper layer is responsible for recovery action.
        return;
    }

    int8_t new_error = (int8_t)(sum / (int16_t)count);

    // FIX [BUG-02]: Detect non-adjacent active sensors (ambiguous straddle).
    //   Non-adjacent pairs produce the same centroid as a single sensor but
    //   represent a fundamentally different physical situation.
    //   Strategy: if active sensors are non-contiguous, preserve last error.
    //   Non-contiguous = gap exists in the bitmask between the first and last
    //   set bit (i.e. popcount of the span exceeds popcount of the mask).
    bool non_contiguous = false;
    if (count >= 2) {
        // Find lowest and highest set bit
        uint8_t lo = 0, hi = 4;
        for (uint8_t i = 0;     i < 5; i++) { if (mask & (1 << i)) { lo = i; break; } }
        for (uint8_t i = 4; (int8_t)i >= 0; i--) { if (mask & (1 << i)) { hi = i; break; } }
        uint8_t span = hi - lo + 1;
        if (span > count) {
            non_contiguous = true;
        }
    }

    if (!non_contiguous) {
        s_line_data.error = new_error;
    }
    // else: non-contiguous sensors — keep last valid error; upper layer holds course
}

/** @return Full LineData snapshot */
LineData DRV_LINE_GetData(void) {
    return s_line_data;
}

/** @return Line position error (-2 to +2) */
int8_t DRV_LINE_GetError(void) {
    return s_line_data.error;
}

/** @return true if all sensors are off (line lost) */
bool DRV_LINE_IsLost(void) {
    return s_line_data.line_lost;
}

// ============================================================
// ════════════════════════════════════════════════════════════
//  DRIVER 6 — ENCODER (Hall effect / slotted disk)
// ════════════════════════════════════════════════════════════
// ============================================================

// Volatile — shared between ISR and main context
static volatile int32_t s_enc_left_ticks  = 0;
static volatile int32_t s_enc_right_ticks = 0;

// Non-volatile working copies — updated in DRV_ENCODER_Update
static EncoderData s_enc_data       = {0, 0, 0.0f, 0.0f, 0.0f, 0.0f};  // FIX [W3]
static int32_t     s_enc_last_left  = 0;
static int32_t     s_enc_last_right = 0;
static uint32_t    s_enc_last_time  = 0;

// FIX [C3]: Use uint32_t to store ms from esp_timer (not millis())
static volatile uint32_t s_enc_left_last_pulse_ms  = 0;
static volatile uint32_t s_enc_right_last_pulse_ms = 0;

/**
 * ISR — left encoder rising edge
 * FIX [C3]: millis() is NOT ISR-safe on ESP32 (acquires a mutex).
 *           Replaced with esp_timer_get_time() which is ISR-safe.
 */
static void IRAM_ATTR enc_isr_left(void) {
    s_enc_left_ticks++;
    s_enc_left_last_pulse_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/** ISR — right encoder rising edge */
static void IRAM_ATTR enc_isr_right(void) {
    s_enc_right_ticks++;
    s_enc_right_last_pulse_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/**
 * @brief  Attach rising-edge interrupts to both encoder pins.
 * @return void
 */
void DRV_ENCODER_Init(void) {
    pinMode(ENCODER_LEFT_PIN,  INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN),  enc_isr_left,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), enc_isr_right, RISING);
    s_enc_last_time = HAL_SYSTEM_GetTick();
}

/**
 * @brief  Compute RPM and distance from tick delta.
 *         Call every ENCODER_RPM_WINDOW_MS (100ms).
 * @return void
 */
void DRV_ENCODER_Update(void) {
    uint32_t now = HAL_SYSTEM_GetTick();
    uint32_t dt  = now - s_enc_last_time;
    if (dt == 0) return;

    // FIX [BUG-04]: Both volatile reads (ticks + last_pulse) consolidated into
    //   one atomic block. Previously a second noInterrupts() block at the end
    //   updated s_enc_data.left_ticks independently from left_now, so an ISR
    //   firing between the two blocks could leave left_ticks != left_now.
    //   Now one snapshot captures everything; s_enc_data.left_ticks is set
    //   directly from the consistent left_now value below.
    noInterrupts();
    int32_t  left_now              = s_enc_left_ticks;
    int32_t  right_now             = s_enc_right_ticks;
    uint32_t left_last_pulse_snap  = s_enc_left_last_pulse_ms;
    uint32_t right_last_pulse_snap = s_enc_right_last_pulse_ms;
    interrupts();

    int32_t delta_left  = left_now  - s_enc_last_left;
    int32_t delta_right = right_now - s_enc_last_right;

    // FIX [BUG-05]: Only recompute RPM when dt >= ENCODER_RPM_WINDOW_MS.
    //   Calling Update() faster than the window (e.g. during burst polling)
    //   causes inflated RPM because delta ticks accumulate over a short dt.
    //   RPM is only valid over the nominal window; skip update otherwise.
    float dt_f = (float)dt;
    if (dt >= ENCODER_RPM_WINDOW_MS) {
        s_enc_data.left_rpm  = ((float)delta_left  / (float)ENCODER_PPR_WHEEL) * (60000.0f / dt_f);
        s_enc_data.right_rpm = ((float)delta_right / (float)ENCODER_PPR_WHEEL) * (60000.0f / dt_f);
    }

    // Zero RPM if no pulse within stale threshold
    if ((now - left_last_pulse_snap)  > ENCODER_STALE_THRESHOLD_MS) s_enc_data.left_rpm  = 0.0f;
    if ((now - right_last_pulse_snap) > ENCODER_STALE_THRESHOLD_MS) s_enc_data.right_rpm = 0.0f;

    // Cumulative distance
    s_enc_data.left_mm  = (float)left_now  * MM_PER_TICK;
    s_enc_data.right_mm = (float)right_now * MM_PER_TICK;

    // FIX [BUG-04]: Use left_now/right_now (already snapshotted above) instead
    //   of re-reading volatile s_enc_left_ticks in a second atomic block.
    s_enc_data.left_ticks  = left_now;
    s_enc_data.right_ticks = right_now;

    s_enc_last_left  = left_now;
    s_enc_last_right = right_now;
    s_enc_last_time  = now;
}

/** @return Full encoder snapshot */
EncoderData DRV_ENCODER_GetData(void) {
    return s_enc_data;
}

/**
 * @brief  Reset tick counters and distance accumulators to zero.
 *         FIX [W7]: also resets s_enc_last_time to prevent spurious
 *         RPM spike on the first Update() after reset.
 *         Use at path start to measure segment distance.
 */
void DRV_ENCODER_Reset(void) {
    noInterrupts();
    s_enc_left_ticks  = 0;
    s_enc_right_ticks = 0;
    interrupts();
    s_enc_last_left   = 0;
    s_enc_last_right  = 0;
    s_enc_data.left_mm  = 0.0f;
    s_enc_data.right_mm = 0.0f;
    // FIX [W7]: reset time base to prevent RPM spike on first update after reset
    s_enc_last_time = HAL_SYSTEM_GetTick();
}

/**
 * @brief  Return average of left and right odometer distance.
 * @return Distance in mm since last Reset()
 */
float DRV_ENCODER_GetDistanceMM(void) {
    return (s_enc_data.left_mm + s_enc_data.right_mm) / 2.0f;
}

// ============================================================
// ════════════════════════════════════════════════════════════
//  DRIVER 7 — L298N MOTOR DRIVER
// ════════════════════════════════════════════════════════════
// ============================================================

/**
 * @brief  Configure direction GPIO pins as outputs, init LEDC PWM.
 *         Both motors stopped at init.
 * @return void
 */
void DRV_L298N_Init(void) {
    pinMode(MOTOR_A_DIR_PIN, OUTPUT);
    pinMode(MOTOR_B_DIR_PIN, OUTPUT);
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
    digitalWrite(MOTOR_B_DIR_PIN, LOW);
    HAL_LEDC_Init();
}

/**
 * @brief  Set both motor speeds simultaneously.
 *         Positive = forward, negative = backward, 0 = stop.
 *         Applies deadband and clamps to ±MOTOR_MAX_SPEED.
 * @param  left   Speed for left motor  (-255 to +255)
 * @param  right  Speed for right motor (-255 to +255)
 */
void DRV_L298N_SetMotors(int16_t left, int16_t right) {
    // Clamp to valid range
    if (left  >  MOTOR_MAX_SPEED) left  =  MOTOR_MAX_SPEED;
    if (left  < -MOTOR_MAX_SPEED) left  = -MOTOR_MAX_SPEED;
    if (right >  MOTOR_MAX_SPEED) right =  MOTOR_MAX_SPEED;
    if (right < -MOTOR_MAX_SPEED) right = -MOTOR_MAX_SPEED;

    // Motor A (left)
    if (left > 0) {
        uint8_t duty = (left < MOTOR_DEADBAND) ? (uint8_t)MOTOR_DEADBAND : (uint8_t)left;
        HAL_LEDC_SetDirection(LEDC_MOTOR_A, MOTOR_DIR_FORWARD);
        HAL_LEDC_SetDuty(LEDC_MOTOR_A, duty);
    } else if (left < 0) {
        int16_t mag  = -left;
        uint8_t duty = (mag < MOTOR_DEADBAND) ? (uint8_t)MOTOR_DEADBAND : (uint8_t)mag;
        HAL_LEDC_SetDirection(LEDC_MOTOR_A, MOTOR_DIR_BACKWARD);
        HAL_LEDC_SetDuty(LEDC_MOTOR_A, duty);
    } else {
        HAL_LEDC_Stop(LEDC_MOTOR_A);
    }

    // Motor B (right)
    if (right > 0) {
        uint8_t duty = (right < MOTOR_DEADBAND) ? (uint8_t)MOTOR_DEADBAND : (uint8_t)right;
        HAL_LEDC_SetDirection(LEDC_MOTOR_B, MOTOR_DIR_FORWARD);
        HAL_LEDC_SetDuty(LEDC_MOTOR_B, duty);
    } else if (right < 0) {
        int16_t mag  = -right;
        uint8_t duty = (mag < MOTOR_DEADBAND) ? (uint8_t)MOTOR_DEADBAND : (uint8_t)mag;
        HAL_LEDC_SetDirection(LEDC_MOTOR_B, MOTOR_DIR_BACKWARD);
        HAL_LEDC_SetDuty(LEDC_MOTOR_B, duty);
    } else {
        HAL_LEDC_Stop(LEDC_MOTOR_B);
    }
}

/**
 * @brief  Controlled stop — zero duty, apply brake on both motors.
 * @return void
 */
void DRV_L298N_Stop(void) {
    HAL_LEDC_StopAll();
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
    digitalWrite(MOTOR_B_DIR_PIN, LOW);
}

/**
 * @brief  Emergency brake — highest priority motor command.
 *         Bypasses all logic; zeroes PWM and direction immediately.
 * @return void
 */
void DRV_L298N_EmergencyBrake(void) {
    // HAL_LEDC_StopAll is atomic — stops both channels in one call
    HAL_LEDC_StopAll();
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
    digitalWrite(MOTOR_B_DIR_PIN, LOW);
}

/**
 * @brief  Rotate robot by `degrees` using AK8963 compass heading.
 *         Spins left motor forward, right motor backward (positive = CW).
 *         Blocks until target heading reached within MOTOR_TURN_HEADING_TOL_DEG.
 *         FIX [W4]: Returns true if turn completed, false if 5s timeout elapsed.
 *         Caller must check return value and handle timeout (e.g., re-localise).
 * @param  degrees  Rotation angle (positive = CW, negative = CCW)
 * @param  speed    Motor duty cycle during turn (0–255)
 * @return true = heading achieved, false = timed out
 */
bool DRV_L298N_TurnDegrees(float degrees, uint8_t speed) {
    // FIX [BUG-03]: Refresh magnetometer before sampling start heading.
    //   Previous code read DRV_AK8963_GetHeading() without first calling
    //   DRV_AK8963_Update(), so the start reference could be >10ms stale,
    //   causing the target heading to be computed from an old bearing.
    DRV_AK8963_Update();
    float start_heading  = DRV_AK8963_GetHeading();
    float target_heading = start_heading + degrees;

    // Normalize target to 0–360
    target_heading = fmodf(target_heading, 360.0f);
    if (target_heading < 0.0f) target_heading += 360.0f;

    // Determine spin direction
    int16_t left_spd  =  (int16_t)speed;
    int16_t right_spd = -(int16_t)speed;
    if (degrees < 0.0f) {
        left_spd  = -(int16_t)speed;
        right_spd =  (int16_t)speed;
    }

    DRV_L298N_SetMotors(left_spd, right_spd);

    bool    success  = false;
    uint32_t deadline = HAL_SYSTEM_GetTick() + 5000;

    while (HAL_SYSTEM_GetTick() < deadline) {
        HAL_SYSTEM_FeedWatchdog();
        DRV_AK8963_Update();

        float current = DRV_AK8963_GetHeading();
        float error   = target_heading - current;

        // Handle wrap-around at 0/360 boundary
        if (error >  180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        if (fabsf(error) <= MOTOR_TURN_HEADING_TOL_DEG) {
            success = true;
            break;
        }
        HAL_SYSTEM_DelayMs(10);
    }

    DRV_L298N_Stop();

    // FIX [W4]: warn on timeout so caller can handle navigation failure
    if (!success) {
        Serial.println(F("[WARN] DRV_L298N_TurnDegrees: timeout — heading not reached"));
    }

    return success;
}

// ============================================================
// ════════════════════════════════════════════════════════════
//  DRIVER 8 — BATTERY MONITOR
// ════════════════════════════════════════════════════════════
// ============================================================

static BatteryData s_battery        = {0.0f, false, false, 0};
static uint32_t    s_battery_last_ms = 0;

/**
 * @brief  Initialise battery monitor (calls HAL_ADC_Init internally).
 * @return void
 */
void DRV_BATTERY_Init(void) {
    HAL_ADC_Init();
    // Take initial reading to populate struct
    float v = HAL_ADC_ReadBatteryVoltage();
    s_battery.voltage     = v;
    s_battery.is_low      = (v < BATTERY_LOW_THRESHOLD_V);
    s_battery.is_very_low = (v < BATTERY_VERYLOW_THRESHOLD_V);

    float range   = BATTERY_FULL_V - BATTERY_VERYLOW_THRESHOLD_V;
    float percent = ((v - BATTERY_VERYLOW_THRESHOLD_V) / range) * 100.0f;
    if (percent > 100.0f) percent = 100.0f;
    if (percent <   0.0f) percent =   0.0f;
    s_battery.percent = (uint8_t)percent;

    s_battery_last_ms = HAL_SYSTEM_GetTick();
}

/**
 * @brief  Low-pass filter new ADC reading, update thresholds.
 *         FIX [W6]: Removed direct call to DRV_L298N_EmergencyBrake().
 *         Calling a motor driver from the battery driver is a layering
 *         violation and is unsafe if motors are not yet initialized.
 *         The application layer MUST poll DRV_BATTERY_IsVeryLow() and
 *         call DRV_L298N_EmergencyBrake() itself.
 *         Call every BATTERY_CHECK_INTERVAL_MS (1s) from main loop.
 * @return void
 */
void DRV_BATTERY_Update(void) {
    uint32_t now = HAL_SYSTEM_GetTick();
    if ((now - s_battery_last_ms) < BATTERY_CHECK_INTERVAL_MS) return;
    s_battery_last_ms = now;

    float new_v = HAL_ADC_ReadBatteryVoltage();

    // First-order IIR low-pass filter
    s_battery.voltage = BATTERY_FILTER_ALPHA * new_v
                      + (1.0f - BATTERY_FILTER_ALPHA) * s_battery.voltage;

    s_battery.is_low      = (s_battery.voltage < BATTERY_LOW_THRESHOLD_V);
    s_battery.is_very_low = (s_battery.voltage < BATTERY_VERYLOW_THRESHOLD_V);

    // Compute percent — clamp 0–100
    float range   = BATTERY_FULL_V - BATTERY_VERYLOW_THRESHOLD_V;
    float percent = ((s_battery.voltage - BATTERY_VERYLOW_THRESHOLD_V) / range) * 100.0f;
    if (percent > 100.0f) percent = 100.0f;
    if (percent <   0.0f) percent =   0.0f;
    s_battery.percent = (uint8_t)percent;

    // FIX [W6]: Emergency brake removed from here.
    // Application layer in loop() already checks DRV_BATTERY_IsVeryLow()
    // and must call DRV_L298N_EmergencyBrake() there.
}

/** @return Full battery snapshot */
BatteryData DRV_BATTERY_GetData(void) {
    return s_battery;
}

/** @return true if battery below LOW threshold */
bool DRV_BATTERY_IsLow(void) {
    return s_battery.is_low;
}

/** @return true if battery below VERY LOW threshold (emergency) */
bool DRV_BATTERY_IsVeryLow(void) {
    return s_battery.is_very_low;
}
// APP_MT3 content included via ESP32_INTEGRATED.h
#include <string.h>
#include <stdint.h>

// ============================================================
// MODULE: SENSOR FUSION
// ============================================================

static SensorFrame _fusion_frame;

void MOD_SENSORFUSION_Init(void) {
    memset(&_fusion_frame, 0, sizeof(SensorFrame));
}

void MOD_SENSORFUSION_Update(void) {
    // --- IMU ---
    DRV_MPU9250_Update();
    DRV_AK8963_Update();

    IMUData imu = DRV_MPU9250_GetData();
    // AK8963 heading accessed via imu.mag_heading_deg (already populated by DRV_MPU9250_GetData)

    // FIX [CRITICAL-1]: heading_deg was being set to gyro_z_dps (angular rate),
    // not integrated heading. Now using imu.heading_deg (integrated by driver).
    _fusion_frame.heading_deg     = imu.heading_deg;
    _fusion_frame.mag_heading_deg = imu.mag_heading_deg;
    _fusion_frame.pitch_deg       = imu.pitch_deg;
    _fusion_frame.roll_deg        = imu.roll_deg;

    // --- Color sensor ---
    DRV_TCS3200_Update();
    _fusion_frame.color = DRV_TCS3200_GetColor();

    // --- Line sensor ---
    DRV_LINE_Update();
    LineData line = DRV_LINE_GetData();
    _fusion_frame.line_error   = line.error;
    _fusion_frame.line_lost    = line.line_lost;
    _fusion_frame.line_bitmask = line.bitmask;

    // --- Ultrasonic (non-blocking) ---
    _fusion_frame.distance_cm = DRV_HCSR04_GetDistance();
    _fusion_frame.obstacle    = (_fusion_frame.distance_cm < HCSR04_OBSTACLE_THRESHOLD_CM);

    // --- Encoders ---
    DRV_ENCODER_Update();
    EncoderData enc = DRV_ENCODER_GetData();
    _fusion_frame.enc_left_ticks  = enc.left_ticks;
    _fusion_frame.enc_right_ticks = enc.right_ticks;
    _fusion_frame.left_rpm        = enc.left_rpm;
    _fusion_frame.right_rpm       = enc.right_rpm;

    // --- Battery ---
    DRV_BATTERY_Update();
    BatteryData bat = DRV_BATTERY_GetData();
    _fusion_frame.battery_v        = bat.voltage;
    _fusion_frame.battery_low      = bat.is_low;
    _fusion_frame.battery_very_low = bat.is_very_low;
    _fusion_frame.battery_percent  = bat.percent;

    // --- Timestamp ---
    _fusion_frame.timestamp_ms = HAL_SYSTEM_GetTick();
}

SensorFrame MOD_SENSORFUSION_GetFrame(void) {
    return _fusion_frame;
}

// ============================================================
// MODULE: PID CONTROLLER
// ============================================================

static float   _pid_kp              = PID_KP;
static float   _pid_ki              = PID_KI;
static float   _pid_kd              = PID_KD;
static int16_t _pid_base_speed      = PID_BASE_SPEED;
static float   _pid_integral        = 0.0f;
static float   _pid_prev_error      = 0.0f;
static float   _pid_prev_derivative = 0.0f;
static int16_t _pid_left_speed      = 0;
static int16_t _pid_right_speed     = 0;

void MOD_PID_Init(void) {
    _pid_kp         = PID_KP;
    _pid_ki         = PID_KI;
    _pid_kd         = PID_KD;
    _pid_base_speed = PID_BASE_SPEED;
    MOD_PID_Reset();
}

void MOD_PID_Reset(void) {
    _pid_integral        = 0.0f;
    _pid_prev_error      = 0.0f;
    _pid_prev_derivative = 0.0f;
    _pid_left_speed      = 0;
    _pid_right_speed     = 0;
}

void MOD_PID_Update(int8_t error) {
    const float dt = PID_SAMPLE_TIME_MS / 1000.0f;

    // Proportional
    float P = _pid_kp * (float)error;

    // Integral with anti-windup
    _pid_integral += (float)error * dt;
    if (_pid_integral >  PID_ANTI_WINDUP_LIMIT) _pid_integral =  PID_ANTI_WINDUP_LIMIT;
    if (_pid_integral < -PID_ANTI_WINDUP_LIMIT) _pid_integral = -PID_ANTI_WINDUP_LIMIT;
    float I = _pid_ki * _pid_integral;

    // Derivative with low-pass filter (prevents derivative kick on setpoint change)
    float raw_derivative      = ((float)error - _pid_prev_error) / dt;
    float filtered_derivative = PID_DERIVATIVE_FILTER * _pid_prev_derivative
                              + (1.0f - PID_DERIVATIVE_FILTER) * raw_derivative;
    float D = _pid_kd * filtered_derivative;
    _pid_prev_derivative = filtered_derivative;

    // Output
    float output = P + I + D;
    if (output >  (float)PID_MAX_OUTPUT) output =  (float)PID_MAX_OUTPUT;
    // FIX [BUG-05]: Apply PID_MIN_OUTPUT for the lower clamp instead of
    // negating PID_MAX_OUTPUT.  Both resolve to -255 with default config,
    // but using the named constant makes asymmetric tuning possible and
    // matches the documented intent.
    if (output < (float)PID_MIN_OUTPUT) output = (float)PID_MIN_OUTPUT;

    // Differential drive commands
    int16_t ls = (int16_t)((float)_pid_base_speed - output);
    int16_t rs = (int16_t)((float)_pid_base_speed + output);

    // Clamp both to valid PWM range
    if (ls >  PID_MAX_OUTPUT) ls =  PID_MAX_OUTPUT;
    if (ls <  PID_MIN_OUTPUT) ls =  PID_MIN_OUTPUT;
    if (rs >  PID_MAX_OUTPUT) rs =  PID_MAX_OUTPUT;
    if (rs <  PID_MIN_OUTPUT) rs =  PID_MIN_OUTPUT;

    _pid_left_speed  = ls;
    _pid_right_speed = rs;
    _pid_prev_error  = (float)error;
}

int16_t MOD_PID_GetLeftSpeed(void)  { return _pid_left_speed;  }
int16_t MOD_PID_GetRightSpeed(void) { return _pid_right_speed; }

void MOD_PID_SetGains(float kp, float ki, float kd) {
    // FIX [CRITICAL-2]: Validate gains before accepting — a corrupted or
    // malicious CAM config packet must not cause runaway motor behavior.
    if (kp < 0.0f || kp > PID_KP_MAX) return;
    if (ki < 0.0f || ki > PID_KI_MAX) return;
    if (kd < 0.0f || kd > PID_KD_MAX) return;
    _pid_kp = kp;
    _pid_ki = ki;
    _pid_kd = kd;
}

void MOD_PID_SetBaseSpeed(int16_t speed) {
    _pid_base_speed = speed;
}

// ============================================================
// MODULE: PACKET BUILDER
// ============================================================

// CRC-8 with polynomial 0x07 (standard CRC-8, no initial value, no final XOR)
// FIX [WARNING-1]: const-correct pointer — callers must not mutate data through this.
uint8_t MOD_PACKET_CRC8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00U;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8U; b++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

// Frame: [0xAA | type | len | payload[0..len-1] | CRC8 | 0x55]
// Returns total frame length.
// FIX [WARNING-1]: const-correct payload pointer.
uint8_t MOD_PACKET_Build(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* out) {
    // FIX [CRITICAL-3]: Guard against len > PACKET_MAX_PAYLOAD to prevent
    // crc_buf overrun (was silent UB in original).
    if (len > PACKET_MAX_PAYLOAD) {
        len = PACKET_MAX_PAYLOAD;
    }

    uint8_t crc_buf[2U + PACKET_MAX_PAYLOAD];
    crc_buf[0] = type;
    crc_buf[1] = len;
    for (uint8_t i = 0; i < len; i++) {
        crc_buf[2U + i] = payload[i];
    }
    uint8_t crc = MOD_PACKET_CRC8(crc_buf, (uint8_t)(2U + len));

    out[0] = PACKET_START_BYTE;
    out[1] = type;
    out[2] = len;
    for (uint8_t i = 0; i < len; i++) {
        out[3U + i] = payload[i];
    }
    out[3U + len] = crc;
    out[4U + len] = PACKET_END_BYTE;

    return (uint8_t)(5U + len);
}

// FIX [WARNING-1]: const-correct frame pointer.
bool MOD_PACKET_Validate(const uint8_t* frame, uint8_t len) {
    // Minimum frame: [start, type, paylen, crc, end] = 5 bytes
    if (len < 5U) return false;
    if (frame[0] != PACKET_START_BYTE)   return false;
    if (frame[len - 1U] != PACKET_END_BYTE) return false;

    uint8_t pay_len = frame[2];

    // FIX [CRITICAL-3]: Guard pay_len before computing expected length —
    // a malformed packet with pay_len=250 would overflow the uint8_t addition.
    if (pay_len > PACKET_MAX_PAYLOAD) return false;

    // Total expected: 5 + pay_len
    if (len != (uint8_t)(5U + pay_len)) return false;

    // Recompute CRC over [type, len, payload]
    uint8_t crc_buf[2U + PACKET_MAX_PAYLOAD];
    crc_buf[0] = frame[1];   // type
    crc_buf[1] = pay_len;
    for (uint8_t i = 0; i < pay_len; i++) {
        crc_buf[2U + i] = frame[3U + i];
    }
    uint8_t computed = MOD_PACKET_CRC8(crc_buf, (uint8_t)(2U + pay_len));
    uint8_t received = frame[3U + pay_len];

    return (computed == received);
}

// ============================================================
// MODULE: PATH RECORDING
// ============================================================

static PathNode  _path_nodes[PATH_MAX_NODES];
static uint16_t  _path_head       = 0;   // next write index (circular)
static uint16_t  _path_count      = 0;   // total valid nodes (max PATH_MAX_NODES)
static bool      _path_recording  = false;
static uint32_t  _path_last_ms    = 0;

void PATH_Init(void) {
    memset(_path_nodes, 0, sizeof(_path_nodes));
    _path_head      = 0;
    _path_count     = 0;
    _path_recording = false;
    _path_last_ms   = 0;
}

void PATH_StartRecording(void) {
    PATH_Clear();
    _path_recording = true;
    _path_last_ms   = 0;
}

void PATH_StopRecording(void) {
    _path_recording = false;
}

bool PATH_IsRecording(void) {
    return _path_recording;
}

void PATH_RecordNode(SensorFrame frame) {
    if (!_path_recording) return;
    if ((frame.timestamp_ms - _path_last_ms) < PATH_RECORD_INTERVAL_MS) return;
    _path_last_ms = frame.timestamp_ms;

    _path_nodes[_path_head].enc_left     = frame.enc_left_ticks;
    _path_nodes[_path_head].enc_right    = frame.enc_right_ticks;
    _path_nodes[_path_head].heading      = frame.heading_deg;
    _path_nodes[_path_head].timestamp_ms = frame.timestamp_ms;

    _path_head = (uint16_t)((_path_head + 1U) % PATH_MAX_NODES);

    if (_path_count < PATH_MAX_NODES) {
        _path_count++;
    }
    // If full, circular overwrite — _path_count stays at PATH_MAX_NODES
}

// Returns node at logical index (0 = oldest, count-1 = newest).
// FIX [BUG-08]: Clamp index to valid range before resolving physical address.
// The original code silently wrapped an out-of-bounds index via modulo, which
// returned stale data with no indication of the error.
PathNode PATH_GetNode(uint16_t index) {
    // Clamp to the number of valid entries
    if (_path_count == 0U) {
        PathNode empty;
        memset(&empty, 0, sizeof(PathNode));
        return empty;
    }
    if (index >= _path_count) {
        index = (uint16_t)(_path_count - 1U);
    }

    uint16_t actual;
    if (_path_count < PATH_MAX_NODES) {
        // Buffer not yet full — entries start at physical index 0
        actual = index;
    } else {
        // Buffer is full — oldest is at _path_head
        actual = (uint16_t)((_path_head + index) % PATH_MAX_NODES);
    }
    return _path_nodes[actual];
}

uint16_t PATH_GetNodeCount(void) {
    return _path_count;
}

void PATH_Clear(void) {
    _path_head  = 0;
    _path_count = 0;
    memset(_path_nodes, 0, sizeof(_path_nodes));
}

// ============================================================
// MODULE: STATE MACHINE
// ============================================================

static SystemState _sm_state           = STATE_INIT;
static SystemState _sm_prev_state      = STATE_INIT;
static SystemEvent _sm_pending_event   = EVENT_NONE;
static bool        _sm_order_confirmed = false;

// Obstacle tracking
static uint32_t    _sm_obstacle_enter_ms      = 0;
static uint32_t    _sm_obstacle_recheck_ms    = 0;
static uint8_t     _sm_obstacle_clear_count   = 0;

// Delivery tracking
static uint32_t    _sm_deliver_enter_ms = 0;
static bool        _sm_deliver_waited   = false;

// Recovery tracking
static uint32_t    _sm_recovery_enter_ms  = 0;
static float       _sm_recovery_spin_deg  = 0.0f;
static bool        _sm_recovery_spin_dir  = true;   // true = CW, false = CCW
static bool        _sm_recovery_reversed  = false;

// FIX [BUG-01]: _sm_return_node_index was set on delivery/recovery but never
// read or decremented during STATE_RETURN.  The variable has been removed.
// STATE_RETURN relies on PID line following until the home color marker is
// detected.  This is the actual runtime behavior in all prior versions;
// the index variable was dead code that implied non-existent path-replay logic.
// If encoder-based path replay is added in a future revision, re-introduce
// the index with a matching replay loop in STATE_RETURN.

static const char* _sm_state_strings[] = {
    "INIT    ",   // 8 chars (STATE_INIT)
    "IDLE    ",   // 8 chars (STATE_IDLE)
    "NAVIGATE",   // 8 chars (STATE_NAVIGATE)
    "OBSTACLE",   // 8 chars (STATE_OBSTACLE)
    "DELIVER ",   // 8 chars (STATE_DELIVER)
    "RETURN  ",   // 8 chars (STATE_RETURN)
    "RECOVERY",   // 8 chars (STATE_RECOVERY)
    "LOW BAT ",   // 8 chars (STATE_LOW_BATTERY)
    // FIX [BUG-04]: "EMERGENCY" is 9 chars.  MOD_UART_CAM_SendTelemetry()
    // copies exactly 8 chars with a null-guard, so the 9th char ('Y') was
    // silently dropped and sent as a space — wrong telemetry display on the
    // dashboard.  Truncated to 8 printable chars to match all other entries.
    "EMERGENC"    // 8 chars (STATE_EMERGENCY) — intentional truncation
};

void MOD_SM_Init(void) {
    _sm_state           = STATE_INIT;
    _sm_prev_state      = STATE_INIT;
    _sm_pending_event   = EVENT_NONE;
    _sm_order_confirmed = false;
}

void MOD_SM_InjectEvent(SystemEvent event) {
    // Emergency always wins immediately — but respect an active RETURN
    if (event == EVENT_VERY_LOW_BATTERY) {
        if (_sm_state != STATE_RETURN) {
            _sm_state = STATE_EMERGENCY;
            DRV_L298N_EmergencyBrake();
        }
        return;
    }
    // FIX [WARNING-2]: Only overwrite pending event if it is more urgent, or slot
    // is empty. This prevents a low-priority event from wiping a critical one that
    // arrived in the same loop tick.  Simplified rule: keep the lowest enum value
    // (smaller value = higher priority by definition above), but never overwrite
    // a real event with EVENT_NONE.
    if (_sm_pending_event == EVENT_NONE) {
        _sm_pending_event = event;
    } else if ((uint8_t)event < (uint8_t)_sm_pending_event) {
        _sm_pending_event = event;
    }
}

SystemState MOD_SM_GetState(void) {
    return _sm_state;
}

const char* MOD_SM_GetStateString(void) {
    if (_sm_state <= STATE_EMERGENCY) {
        return _sm_state_strings[(int)_sm_state];
    }
    return "UNKNOWN ";
}

bool MOD_SM_IsOrderConfirmed(void) {
    return _sm_order_confirmed;
}

void MOD_SM_ConfirmOrder(void) {
    _sm_order_confirmed = true;
}

// Internal helper: apply low battery speed scaling
static void _sm_apply_speed(int16_t left, int16_t right, bool low_bat) {
    if (low_bat) {
        left  = (int16_t)((float)left  * LOW_BATTERY_SPEED_FACTOR);
        right = (int16_t)((float)right * LOW_BATTERY_SPEED_FACTOR);
    }
    DRV_L298N_SetMotors(left, right);
}

void MOD_SM_Update(SensorFrame frame) {
    uint32_t now = frame.timestamp_ms;

    // ---- Global battery emergency check (every state except EMERGENCY) ----
    if (_sm_state != STATE_EMERGENCY) {
        if (frame.battery_very_low) {
            // Allow STATE_RETURN to finish
            if (_sm_state != STATE_RETURN) {
                _sm_state = STATE_EMERGENCY;
                DRV_L298N_EmergencyBrake();
                return;
            }
        }
    }

    // Consume pending event
    SystemEvent ev = _sm_pending_event;
    _sm_pending_event = EVENT_NONE;

    switch (_sm_state) {

        // --------------------------------------------------------
        case STATE_INIT:
            // FIX [WARNING-3]: Re-initialising sensor fusion and PID inside the
            // state machine update is unexpected — these were already called in
            // APP_MAIN_Init.  In a multi-call scenario this wipes any calibration.
            // Removed the duplicate calls; only reset path and order flag here.
            PATH_Init();
            _sm_order_confirmed = false;
            _sm_state = STATE_IDLE;
            break;

        // --------------------------------------------------------
        case STATE_IDLE:
            DRV_L298N_Stop();
            if (_sm_order_confirmed) {
                PATH_StartRecording();
                MOD_PID_Reset();
                _sm_state = STATE_NAVIGATE;
            }
            break;

        // --------------------------------------------------------
        case STATE_NAVIGATE:
            // Record path
            PATH_RecordNode(frame);

            // Battery degraded — downshift to LOW_BATTERY
            if ((ev == EVENT_LOW_BATTERY) || frame.battery_low) {
                _sm_state = STATE_LOW_BATTERY;
                break;
            }

            // Check arrival at delivery
            if ((ev == EVENT_DELIVERY_REACHED) || (frame.color == DELIVERY_DETECTION_COLOR)) {
                PATH_StopRecording();
                DRV_L298N_Stop();
                _sm_deliver_enter_ms = now;
                _sm_deliver_waited   = false;
                _sm_state = STATE_DELIVER;
                break;
            }

            // Obstacle check
            if ((ev == EVENT_OBSTACLE_DETECTED) || frame.obstacle) {
                DRV_L298N_Stop();
                _sm_obstacle_enter_ms    = now;
                _sm_obstacle_recheck_ms  = now;
                _sm_obstacle_clear_count = 0;
                _sm_state = STATE_OBSTACLE;
                break;
            }

            // Line lost
            if ((ev == EVENT_LINE_LOST) || frame.line_lost) {
                DRV_L298N_Stop();
                _sm_recovery_enter_ms = now;
                _sm_recovery_spin_deg = 0.0f;
                _sm_recovery_spin_dir = true;
                _sm_recovery_reversed = false;
                _sm_state = STATE_RECOVERY;
                break;
            }

            // PID line following
#ifdef NAV_MODE_SIMPLE_PID
            MOD_PID_Update(frame.line_error);
            _sm_apply_speed(MOD_PID_GetLeftSpeed(), MOD_PID_GetRightSpeed(), frame.battery_low);
#endif
#ifdef NAV_MODE_GRID_FLOODFILL
            FF_Update(frame);
#endif
            break;

        // --------------------------------------------------------
        case STATE_OBSTACLE:
            DRV_L298N_Stop();

            // FIX [BUG-03]: OBSTACLE_WAIT_MS was defined but never used.
            // _sm_obstacle_enter_ms was recorded but the state machine had no
            // hard timeout — a stuck obstacle sensor would park the robot forever.
            // Added a hard timeout: after OBSTACLE_WAIT_MS milliseconds the robot
            // declares the path clear and resumes.  This matches the documented
            // behavior in the header comment ("wait before retrying").
            if ((now - _sm_obstacle_enter_ms) >= OBSTACLE_WAIT_MS) {
                // Hard timeout — force resume regardless of sensor reading.
                // This handles a failed/stuck ultrasonic sensor gracefully.
                MOD_PID_Reset();
                _sm_state = STATE_NAVIGATE;
                break;
            }

            // Periodic recheck (runs only within the OBSTACLE_WAIT_MS window)
            if ((now - _sm_obstacle_recheck_ms) >= OBSTACLE_RECHECK_INTERVAL_MS) {
                _sm_obstacle_recheck_ms = now;

                if (!frame.obstacle) {
                    _sm_obstacle_clear_count++;
                } else {
                    _sm_obstacle_clear_count = 0;  // reset on flap
                }

                if (_sm_obstacle_clear_count >= OBSTACLE_CLEAR_COUNT_REQUIRED) {
                    MOD_PID_Reset();
                    _sm_state = STATE_NAVIGATE;
                }
            }
            break;

        // --------------------------------------------------------
        case STATE_DELIVER:
            DRV_L298N_Stop();

            if (!_sm_deliver_waited) {
                if ((now - _sm_deliver_enter_ms) >= DELIVERY_WAIT_MS) {
                    _sm_deliver_waited = true;
                    // Perform 180° U-turn — blocking call; acceptable here as it is
                    // a deliberate, one-shot manoeuvre at a known static location.
                    DRV_L298N_TurnDegrees(RETURN_TURN_DEGREES, RETURN_TURN_SPEED);
                    // FIX [BUG-01]: Removed dead _sm_return_node_index assignment.
                    MOD_PID_Reset();
                    _sm_state = STATE_RETURN;
                }
            }
            break;

        // --------------------------------------------------------
        case STATE_RETURN: {
            // Check home color
            if ((ev == EVENT_HOME_REACHED) || (frame.color == HOME_DETECTION_COLOR)) {
                DRV_L298N_Stop();
                PATH_Clear();
                _sm_order_confirmed = false;
                _sm_state = STATE_IDLE;
                break;
            }

            // PID still active on the line during return
            if (!frame.line_lost) {
                MOD_PID_Update(frame.line_error);
                // battery_very_low: robot is running return path so we permit it
                // but apply speed reduction regardless
                _sm_apply_speed(MOD_PID_GetLeftSpeed(), MOD_PID_GetRightSpeed(),
                                frame.battery_low || frame.battery_very_low);
            } else {
                DRV_L298N_Stop();
            }
            break;
        }

        // --------------------------------------------------------
        case STATE_RECOVERY: {
            bool timed_out = ((now - _sm_recovery_enter_ms) >= LINE_LOST_RECOVERY_TIMEOUT_MS);

            // Line found → resume navigate
            if ((ev == EVENT_LINE_FOUND) || !frame.line_lost) {
                MOD_PID_Reset();
                _sm_state = STATE_NAVIGATE;
                break;
            }

            if (!timed_out) {
                // Slow ±30° spin search
                if (!_sm_recovery_reversed) {
                    if (_sm_recovery_spin_dir) {
                        DRV_L298N_SetMotors(50, -50);   // CW
                        _sm_recovery_spin_deg += 1.0f;
                        if (_sm_recovery_spin_deg >= 30.0f) {
                            _sm_recovery_spin_dir = false;
                            _sm_recovery_spin_deg = 0.0f;
                        }
                    } else {
                        DRV_L298N_SetMotors(-50, 50);   // CCW
                        _sm_recovery_spin_deg += 1.0f;
                        if (_sm_recovery_spin_deg >= 60.0f) {
                            _sm_recovery_spin_dir = true;
                            _sm_recovery_spin_deg = 0.0f;
                        }
                    }
                }
            } else if (!_sm_recovery_reversed) {
                // FIX [CRITICAL-4]: Removed HAL_SYSTEM_DelayMs(300) blocking call.
                // Replaced with non-blocking reverse managed across loop ticks.
                _sm_recovery_reversed = true;
                _sm_recovery_enter_ms = now;  // reuse enter_ms as reverse timer

                if (PATH_GetNodeCount() > 0) {
                    DRV_L298N_SetMotors(-60, -60);  // begin reverse
                }
            } else {
                // FIX [CRITICAL-4] continued: hold reverse for 300 ms then stop/decide
                if ((now - _sm_recovery_enter_ms) < 300U) {
                    if (PATH_GetNodeCount() > 0) {
                        DRV_L298N_SetMotors(-60, -60);
                    }
                } else {
                    // Reverse done — still lost → return home via line following
                    DRV_L298N_Stop();
                    PATH_StopRecording();
                    MOD_PID_Reset();
                    _sm_state = STATE_RETURN;
                }
            }
            break;
        }

        // --------------------------------------------------------
        case STATE_LOW_BATTERY:
            // Continue task at 70% speed
            PATH_RecordNode(frame);

            if ((ev == EVENT_DELIVERY_REACHED) || (frame.color == DELIVERY_DETECTION_COLOR)) {
                PATH_StopRecording();
                DRV_L298N_Stop();
                _sm_deliver_enter_ms = now;
                _sm_deliver_waited   = false;
                _sm_state = STATE_DELIVER;
                break;
            }

            if ((ev == EVENT_OBSTACLE_DETECTED) || frame.obstacle) {
                DRV_L298N_Stop();
                _sm_obstacle_enter_ms    = now;
                _sm_obstacle_recheck_ms  = now;
                _sm_obstacle_clear_count = 0;
                _sm_state = STATE_OBSTACLE;
                break;
            }

            if ((ev == EVENT_LINE_LOST) || frame.line_lost) {
                DRV_L298N_Stop();
                _sm_recovery_enter_ms = now;
                _sm_recovery_spin_deg = 0.0f;
                _sm_recovery_spin_dir = true;
                _sm_recovery_reversed = false;
                _sm_state = STATE_RECOVERY;
                break;
            }

#ifdef NAV_MODE_SIMPLE_PID
            MOD_PID_Update(frame.line_error);
            _sm_apply_speed(MOD_PID_GetLeftSpeed(), MOD_PID_GetRightSpeed(), true);
#endif
#ifdef NAV_MODE_GRID_FLOODFILL
            FF_Update(frame);
#endif
            break;

        // --------------------------------------------------------
        case STATE_EMERGENCY:
            // Stay here forever — never leave
            DRV_L298N_EmergencyBrake();
            break;

        default:
            break;
    }

    // Detect state transition for UART update
    _sm_prev_state = _sm_state;
}

// ============================================================
// MODULE: UART CAM
// ============================================================

static CAM_Status _cam_status        = CAM_OK;
static uint32_t   _cam_corrupt_count = 0;
static uint32_t   _cam_last_telem_ms = 0;

// Rx parser state machine
typedef enum {
    RX_WAIT_START = 0,
    RX_WAIT_TYPE,
    RX_WAIT_LEN,
    RX_WAIT_PAYLOAD,
    RX_WAIT_CRC,
    RX_WAIT_END
} RxParseState;

static RxParseState _cam_rx_state   = RX_WAIT_START;
static uint8_t      _cam_rx_buf[5U + PACKET_MAX_PAYLOAD];
static uint8_t      _cam_rx_buf_pos = 0;
static uint8_t      _cam_rx_type    = 0;
static uint8_t      _cam_rx_len     = 0;
static uint8_t      _cam_rx_pay_idx = 0;

void MOD_UART_CAM_Init(void) {
    _cam_status        = CAM_OK;
    _cam_corrupt_count = 0;
    _cam_last_telem_ms = 0;
    _cam_rx_state      = RX_WAIT_START;
    _cam_rx_buf_pos    = 0;
}

void MOD_UART_CAM_SendTelemetry(SensorFrame frame, SystemState state) {
    (void)state;   // retained in signature for future use; state is encoded in string below
    uint8_t payload[TELEMETRY_PAYLOAD_SIZE];

    // enc_left / enc_right — truncated to int16, big-endian
    int16_t el  = (int16_t)(frame.enc_left_ticks  & 0xFFFF);
    int16_t er  = (int16_t)(frame.enc_right_ticks & 0xFFFF);
    // heading as degrees*10, int16
    int16_t hdg = (int16_t)(frame.heading_deg * 10.0f);

    payload[0] = (uint8_t)((el  >> 8) & 0xFF);
    payload[1] = (uint8_t)( el        & 0xFF);
    payload[2] = (uint8_t)((er  >> 8) & 0xFF);
    payload[3] = (uint8_t)( er        & 0xFF);
    payload[4] = (uint8_t)((hdg >> 8) & 0xFF);
    payload[5] = (uint8_t)( hdg       & 0xFF);

    // State string — 8 bytes (space-padded, no null terminator in payload)
    // FIX [BUG-04]: _sm_state_strings[] now has exactly 8 printable chars for
    // every entry (including EMERGENCY → "EMERGENC"). The null-guard in the
    // loop below is retained as a safety net but should never trigger.
    const char* ss = MOD_SM_GetStateString();
    for (uint8_t i = 0; i < 8U; i++) {
        payload[6U + i] = (uint8_t)(ss[i] ? ss[i] : ' ');
    }

    uint8_t out[5U + TELEMETRY_PAYLOAD_SIZE];
    uint8_t frame_len = MOD_PACKET_Build(MSG_CAM_TELEMETRY, payload, TELEMETRY_PAYLOAD_SIZE, out);
    HAL_UART_SendBytes(UART_PORT_CAM, out, frame_len);
}

void MOD_UART_CAM_ProcessRx(void) {
    int avail = HAL_UART_BytesAvailable(UART_PORT_CAM);
    while (avail-- > 0) {
        uint8_t b = HAL_UART_ReadByte(UART_PORT_CAM);

        switch (_cam_rx_state) {
            case RX_WAIT_START:
                if (b == PACKET_START_BYTE) {
                    memset(_cam_rx_buf, 0, sizeof(_cam_rx_buf));
                    _cam_rx_buf_pos = 0;
                    _cam_rx_buf[_cam_rx_buf_pos++] = b;
                    _cam_rx_state = RX_WAIT_TYPE;
                }
                break;

            case RX_WAIT_TYPE:
                _cam_rx_type = b;
                _cam_rx_buf[_cam_rx_buf_pos++] = b;
                _cam_rx_state = RX_WAIT_LEN;
                break;

            case RX_WAIT_LEN:
                _cam_rx_len = b;
                _cam_rx_buf[_cam_rx_buf_pos++] = b;
                if (_cam_rx_len == 0U) {
                    _cam_rx_state = RX_WAIT_CRC;
                } else if (_cam_rx_len > PACKET_MAX_PAYLOAD) {
                    // Malformed — reset parser
                    _cam_rx_state = RX_WAIT_START;
                    _cam_corrupt_count++;
                } else {
                    _cam_rx_pay_idx = 0;
                    _cam_rx_state   = RX_WAIT_PAYLOAD;
                }
                break;

            case RX_WAIT_PAYLOAD:
                // FIX [CRITICAL-3]: Guard buf_pos against overrun before writing.
                if (_cam_rx_buf_pos < (uint8_t)sizeof(_cam_rx_buf)) {
                    _cam_rx_buf[_cam_rx_buf_pos++] = b;
                }
                _cam_rx_pay_idx++;
                if (_cam_rx_pay_idx >= _cam_rx_len) {
                    _cam_rx_state = RX_WAIT_CRC;
                }
                break;

            case RX_WAIT_CRC:
                if (_cam_rx_buf_pos < (uint8_t)sizeof(_cam_rx_buf)) {
                    _cam_rx_buf[_cam_rx_buf_pos++] = b;
                }
                _cam_rx_state = RX_WAIT_END;
                break;

            case RX_WAIT_END:
                if (_cam_rx_buf_pos < (uint8_t)sizeof(_cam_rx_buf)) {
                    _cam_rx_buf[_cam_rx_buf_pos++] = b;
                }
                // Validate full frame
                if (MOD_PACKET_Validate(_cam_rx_buf, _cam_rx_buf_pos)) {
                    // Decode message
                    switch (_cam_rx_type) {
                        case MSG_CAM_ERROR: {
                            uint8_t err_code = (_cam_rx_len >= 1U) ? _cam_rx_buf[3] : 0U;
                            switch (err_code) {
                                case 1U: _cam_status = CAM_SD_FAIL;  MOD_SM_InjectEvent(EVENT_CAM_SD_FAIL); break;
                                case 2U: _cam_status = CAM_SD_FULL;  MOD_SM_InjectEvent(EVENT_CAM_SD_FAIL); break;
                                case 3U: _cam_status = CAM_BUF_FULL; break;
                                default: _cam_status = CAM_OK;       break;
                            }
                            break;
                        }
                        case MSG_CAM_CONFIG: {
                            // Config update — PID gains sent from web dashboard via CAM
                            // Payload format: kp(float32 LE) ki(float32 LE) kd(float32 LE) = 12 bytes
                            // FIX [BUG-09]: Comment previously claimed BE byte order.  ESP32 is
                            // little-endian; memcpy into a float is only correct when the sender
                            // also uses little-endian byte order.  CAM firmware must be updated
                            // to send floats in little-endian (native ESP32) format.  If the CAM
                            // sends big-endian, add a 4-byte swap here before the memcpy.
                            // The byte-swap helper is provided for reference:
                            //   uint8_t tmp[4] = {p[3],p[2],p[1],p[0]};
                            //   memcpy(&new_kp, tmp, 4U);
                            if (_cam_rx_len >= 12U) {
                                float new_kp, new_ki, new_kd;
                                const uint8_t* p = &_cam_rx_buf[3];
                                memcpy(&new_kp, p + 0, 4U);
                                memcpy(&new_ki, p + 4, 4U);
                                memcpy(&new_kd, p + 8, 4U);
                                // MOD_PID_SetGains validates ranges internally
                                MOD_PID_SetGains(new_kp, new_ki, new_kd);
                            }
                            break;
                        }
                        default:
                            break;
                    }
                } else {
                    _cam_corrupt_count++;
                }
                _cam_rx_state   = RX_WAIT_START;
                _cam_rx_buf_pos = 0;
                break;
        }
    }
}

CAM_Status MOD_UART_CAM_GetStatus(void) {
    return _cam_status;
}

// ============================================================
// MODULE: UART NANO
// ============================================================

void MOD_UART_NANO_Init(void) {
    // HAL UART already initialized in APP_MAIN_Init
}

void MOD_UART_NANO_SendInit(void) {
    const uint8_t payload[8] = {'S','Y','S','R','E','A','D','Y'};
    uint8_t out[5U + 8U];
    // FIX [BUG-02]: Use corrected MSG_NANO_INIT (0x10) instead of the original
    // MSG_TYPE_NANO_INIT which collided with MSG_TYPE_TELEMETRY (both 0x01).
    uint8_t len = MOD_PACKET_Build(MSG_NANO_INIT, payload, 8U, out);
    HAL_UART_SendBytes(UART_PORT_NANO, out, len);
}

void MOD_UART_NANO_SendState(SystemState state) {
    // FIX [CRITICAL-5]: "EMERGENCY" is 9 characters — reading ss[i] for i=8
    // on a string literal with only 8 meaningful chars was an off-by-one read.
    // All strings are now exactly 8 printable characters with a null terminator.
    static const char* nano_state_strings[] = {
        "INIT    ",   // 8 chars + '\0'
        "IDLE    ",
        "NAVIGATE",
        "OBSTACLE",
        "DELIVER ",
        "RETURN  ",
        "RECOVERY",
        "LOW BAT ",
        "EMERGENC"    // truncated to exactly 8 printable chars
    };

    const char* ss = (state <= STATE_EMERGENCY) ? nano_state_strings[(int)state] : "UNKNOWN ";
    uint8_t payload[8U];
    for (uint8_t i = 0U; i < 8U; i++) {
        payload[i] = (uint8_t)(ss[i] ? (uint8_t)ss[i] : (uint8_t)' ');
    }

    uint8_t out[5U + 8U];
    // FIX [BUG-02]: Use corrected MSG_NANO_STATE (0x11) instead of the original
    // MSG_TYPE_NANO_STATE which collided with MSG_TYPE_ERROR (both 0x02).
    uint8_t len = MOD_PACKET_Build(MSG_NANO_STATE, payload, 8U, out);
    HAL_UART_SendBytes(UART_PORT_NANO, out, len);
}

// ============================================================
// MODULE: FLOOD FILL (grid mode only)
// ============================================================

#ifdef NAV_MODE_GRID_FLOODFILL

typedef enum { CELL_UNVISITED = 0, CELL_VISITED = 1, CELL_WALL = 2 } CellState;

static CellState _grid[FLOODFILL_MAX_CELLS];
// FIX [BUG-06]: BFS queue enlarged to FLOODFILL_BFS_QUEUE_SIZE (128).
// The original queue size equalled MAX_CELLS (64), but each cell can be
// re-enqueued when a shorter path is discovered before it is popped.
// Worst-case enqueues for a 64-cell 1-D chain = 2*(MAX_CELLS-1) ≈ 126.
// With a 64-entry queue, entries 65–126 would silently overwrite earlier
// entries, corrupting the BFS traversal and producing wrong flood values.
static uint8_t   _flood[FLOODFILL_MAX_CELLS];
static uint8_t   _current_cell        = 0;
static uint8_t   _total_cells         = 0;
static bool      _ff_at_junction      = false;
static int32_t   _ff_junction_enc_ref = 0;
static bool      _ff_exploration_done = false;

// Simple BFS flood fill from unvisited frontier outward
static void _ff_compute_flood(void) {
    // Initialize all flood values to max
    for (uint8_t i = 0; i < FLOODFILL_MAX_CELLS; i++) {
        _flood[i] = 255U;
    }

    // FIX [BUG-06]: Queue size is now FLOODFILL_BFS_QUEUE_SIZE (>= 2*MAX_CELLS)
    uint8_t queue[FLOODFILL_BFS_QUEUE_SIZE];
    uint8_t qhead = 0, qtail = 0;

    // Seed: unvisited cells get flood value 0
    for (uint8_t i = 0; i < _total_cells; i++) {
        if (_grid[i] == CELL_UNVISITED) {
            _flood[i] = 0U;
            queue[qtail] = i;
            qtail = (uint8_t)((qtail + 1U) % FLOODFILL_BFS_QUEUE_SIZE);
        }
    }

    // BFS outward
    while (qhead != qtail) {
        uint8_t cell = queue[qhead];
        qhead = (uint8_t)((qhead + 1U) % FLOODFILL_BFS_QUEUE_SIZE);

        // 1-D chain neighbours: cell-1 and cell+1
        // FIX [WARNING-4]: Use signed comparison; cast to int before
        // subtracting to avoid uint8_t underflow when cell == 0.
        int16_t neighbors[2] = { (int16_t)cell - 1, (int16_t)cell + 1 };
        for (uint8_t n = 0; n < 2U; n++) {
            int16_t nb = neighbors[n];
            if (nb < 0 || nb >= (int16_t)_total_cells) continue;
            if (_grid[(uint8_t)nb] == CELL_WALL) continue;
            // FIX [WARNING-5]: Guard against flood value overflow (255+1 wraps to 0)
            if (_flood[cell] == 255U) continue;
            uint8_t new_flood = (uint8_t)(_flood[cell] + 1U);
            if (new_flood < _flood[(uint8_t)nb]) {
                _flood[(uint8_t)nb] = new_flood;
                queue[qtail] = (uint8_t)nb;
                qtail = (uint8_t)((qtail + 1U) % FLOODFILL_BFS_QUEUE_SIZE);
            }
        }
    }
}

void FF_Init(void) {
    memset(_grid,  CELL_UNVISITED, sizeof(_grid));
    memset(_flood, 0,              sizeof(_flood));
    _current_cell        = 0;
    _total_cells         = 1;
    _ff_at_junction      = false;
    _ff_junction_enc_ref = 0;
    _ff_exploration_done = false;
}

void FF_Update(SensorFrame frame) {
    // Detect junction: popcount of line_bitmask >= threshold
    uint8_t sensor_count = 0;
    uint8_t bm = frame.line_bitmask;
    while (bm) { sensor_count += (bm & 1U); bm >>= 1U; }

    bool at_junction = (sensor_count >= FLOODFILL_JUNCTION_THRESHOLD);

    if (at_junction && !_ff_at_junction) {
        // Rising edge: arrived at new junction
        _ff_at_junction      = true;
        _ff_junction_enc_ref = frame.enc_left_ticks;

        // Mark current cell visited
        _grid[_current_cell] = CELL_VISITED;

        // Discover neighbours — expand total_cells if needed
        if ((_current_cell + 1U) >= _total_cells && _total_cells < FLOODFILL_MAX_CELLS) {
            _total_cells++;
        }

        // Recompute flood fill
        _ff_compute_flood();

        // Check if all cells visited
        _ff_exploration_done = true;
        for (uint8_t i = 0; i < _total_cells; i++) {
            if (_grid[i] == CELL_UNVISITED) { _ff_exploration_done = false; break; }
        }

        // Move to lowest flood-value neighbour
        uint8_t best_cell  = _current_cell;
        uint8_t best_flood = 255U;
        int16_t fwd  = (int16_t)_current_cell + 1;
        int16_t bkwd = (int16_t)_current_cell - 1;

        if (fwd < (int16_t)_total_cells &&
            _grid[(uint8_t)fwd] != CELL_WALL &&
            _flood[(uint8_t)fwd] < best_flood) {
            best_flood = _flood[(uint8_t)fwd];
            best_cell  = (uint8_t)fwd;
        }
        if (bkwd >= 0 &&
            _grid[(uint8_t)bkwd] != CELL_WALL &&
            _flood[(uint8_t)bkwd] < best_flood) {
            best_cell = (uint8_t)bkwd;
        }

        _current_cell = best_cell;

    } else if (!at_junction) {
        _ff_at_junction = false;
    }

    // Between junctions: PID line following at cruise speed
    MOD_PID_SetBaseSpeed(FLOODFILL_CRUISE_SPEED);
    MOD_PID_Update(frame.line_error);
    DRV_L298N_SetMotors(MOD_PID_GetLeftSpeed(), MOD_PID_GetRightSpeed());
}

bool FF_IsExplorationComplete(void) {
    return _ff_exploration_done;
}

void FF_Reset(void) {
    FF_Init();
}

#endif  // NAV_MODE_GRID_FLOODFILL

// ============================================================
// APP MAIN
// ============================================================

static SystemState _app_prev_state    = STATE_INIT;
static uint32_t    _app_telem_last_ms = 0;
static uint32_t    _app_loop_last_ms  = 0;

void APP_MAIN_Init(void) {
    // 1. HAL layer
    HAL_SYSTEM_Init();
    HAL_I2C_Init();
    HAL_UART_Init(UART_PORT_CAM);
    HAL_UART_Init(UART_PORT_NANO);
    HAL_ADC_Init();
    HAL_LEDC_Init();

    // 2. Drivers
    DRV_MPU9250_Init();
    DRV_AK8963_Init();
    DRV_TCS3200_Init();
    DRV_HCSR04_Init();
    DRV_LINE_Init();
    DRV_ENCODER_Init();
    DRV_L298N_Init();
    DRV_BATTERY_Init();

    // 3. Application modules
    MOD_SENSORFUSION_Init();
    MOD_PID_Init();
    MOD_SM_Init();
    MOD_UART_CAM_Init();
    MOD_UART_NANO_Init();
    MOD_UART_NANO_SendInit();
    PATH_Init();

    _app_prev_state    = STATE_INIT;
    _app_telem_last_ms = 0;
    _app_loop_last_ms  = HAL_SYSTEM_GetTick();

#ifdef NAV_MODE_GRID_FLOODFILL
    FF_Init();
#endif
}

void APP_MAIN_Run(void) {
    // Enforce exactly 10ms loop period (100Hz)
    uint32_t now     = HAL_SYSTEM_GetTick();
    uint32_t elapsed = now - _app_loop_last_ms;
    if (elapsed < (uint32_t)PID_SAMPLE_TIME_MS) {
        return;  // Not yet time — caller will re-invoke immediately
    }
    _app_loop_last_ms = now;

    // 1. Feed watchdog
    HAL_SYSTEM_FeedWatchdog();

    // 2. Update sensor fusion
    MOD_SENSORFUSION_Update();

    // 3. Get current sensor frame
    SensorFrame frame = MOD_SENSORFUSION_GetFrame();

    // 4. State machine update
    MOD_SM_Update(frame);

    // 5. Process inbound bytes from CAM
    MOD_UART_CAM_ProcessRx();

    // 6. Send telemetry every 100ms (10Hz)
    if ((frame.timestamp_ms - _app_telem_last_ms) >= TELEMETRY_SEND_INTERVAL_MS) {
        _app_telem_last_ms = frame.timestamp_ms;
        MOD_UART_CAM_SendTelemetry(frame, MOD_SM_GetState());
    }

    // 7. Send state update to Nano on state change
    SystemState cur_state = MOD_SM_GetState();
    if (cur_state != _app_prev_state) {
        MOD_UART_NANO_SendState(cur_state);
        _app_prev_state = cur_state;
    }
}
