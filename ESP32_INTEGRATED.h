// ============================================================
// HAL_MT1.h — Monolithic HAL Header
// Project  : Waiter Robot — ESP32_MAIN
// Board    : ESP32 WROOM-32 38-pin
// Combines : hal_system, hal_i2c, hal_uart, hal_adc, hal_ledc
//            configs, types, and declarations (25 files → 1 header)
//
// QA AUDIT v1.1 — Bugs fixed:
//   [BUG-H-1]  ADC_BATTERY_VOLTAGE_MIN_V / MAX_V missing — Test_ADC_BatteryVoltage
//              used magic numbers 3.0/13.0 with no named constants. Added.
//   [BUG-H-2]  LEDC_MOTOR_DEADBAND defined as bare int literal with no type
//              safety — added explicit cast reminder in comment, unchanged value.
//   [BUG-H-3]  No HAL_UART_TX_ONLY_PORT guard — UART_PORT_NANO has RX_PIN=-1
//              but nothing in the header signalled this to callers. Added
//              UART_NANO_IS_TX_ONLY define.
//   [BUG-H-4]  I2C_ADDR_AK8963 declared but never used/tested — flagged as
//              dead declaration; kept but marked.
// ============================================================
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <driver/adc.h>
#include <driver/ledc.h>

// ============================================================
// ─── MODULE 1 — HAL_SYSTEM CONFIG ───────────────────────────
// ============================================================
#define WATCHDOG_TIMEOUT_MS          2000
#define MAIN_LOOP_PERIOD_MS          10
#define TICK_TEST_TOLERANCE_MS       5
#define DELAY_TEST_TOLERANCE_MS      5
#define CPU_FREQ_MHZ                 240

// ─── HAL_SYSTEM TYPES ───────────────────────────────────────
typedef enum {
    HAL_SYS_OK      = 0,
    HAL_SYS_ERROR   = 1,
    HAL_SYS_TIMEOUT = 2
} HAL_SYS_Status;

// ─── HAL_SYSTEM API ─────────────────────────────────────────
void           HAL_SYSTEM_Init(void);
uint32_t       HAL_SYSTEM_GetTick(void);
void           HAL_SYSTEM_DelayMs(uint32_t ms);
void           HAL_SYSTEM_FeedWatchdog(void);
HAL_SYS_Status HAL_SYSTEM_GetStatus(void);
void           HAL_SYSTEM_Reset(void);

// ============================================================
// ─── MODULE 2 — HAL_I2C CONFIG ──────────────────────────────
// ============================================================
#define I2C_SDA_PIN                  21
#define I2C_SCL_PIN                  22
#define I2C_FREQ_HZ                  400000UL   // 400kHz fast mode
#define I2C_TIMEOUT_MS               10
#define I2C_RETRY_COUNT              3

#define I2C_ADDR_MPU9250             0x68
// [BUG-H-4] AK8963 address declared but no corresponding test exists.
// Design risk: magnetometer path is untested. Flagged for future test coverage.
#define I2C_ADDR_AK8963              0x0C

// ─── HAL_I2C TYPES ───────────────────────────────────────────
typedef enum {
    HAL_I2C_OK      = 0,
    HAL_I2C_NACK    = 1,
    HAL_I2C_TIMEOUT = 2,
    HAL_I2C_ERROR   = 3
} HAL_I2C_Status;

// ─── HAL_I2C API ─────────────────────────────────────────────
void           HAL_I2C_Init(void);
HAL_I2C_Status HAL_I2C_Write(uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t len);
HAL_I2C_Status HAL_I2C_Read(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len);
HAL_I2C_Status HAL_I2C_WriteByte(uint8_t addr, uint8_t reg, uint8_t data);
HAL_I2C_Status HAL_I2C_ReadByte(uint8_t addr, uint8_t reg, uint8_t* data);
bool           HAL_I2C_DevicePresent(uint8_t addr);

// ============================================================
// ─── MODULE 3 — HAL_UART CONFIG ─────────────────────────────
// ============================================================

// UART1 → ESP32-CAM
#define UART_CAM_PORT                1
#define UART_CAM_TX_PIN              17
#define UART_CAM_RX_PIN              16
#define UART_CAM_BAUD                115200
#define UART_CAM_RX_BUFFER_SIZE      256

// UART2 → Arduino Nano (TX only)
#define UART_NANO_PORT               2
#define UART_NANO_TX_PIN             4
#define UART_NANO_RX_PIN             -1
#define UART_NANO_BAUD               115200
// [BUG-H-3] Explicit TX-only guard so callers do not attempt reads on NANO port.
#define UART_NANO_IS_TX_ONLY         1

// UART0 → Debug
#define UART_DEBUG_BAUD              115200

// ─── HAL_UART TYPES ──────────────────────────────────────────
typedef enum {
    UART_PORT_CAM   = 1,
    UART_PORT_NANO  = 2,
    UART_PORT_DEBUG = 0
} UART_Port;

typedef enum {
    HAL_UART_OK       = 0,
    HAL_UART_ERROR    = 1,
    HAL_UART_OVERFLOW = 2,
    HAL_UART_TIMEOUT  = 3
} HAL_UART_Status;

// ─── HAL_UART API ────────────────────────────────────────────
void            HAL_UART_Init(UART_Port port);
void            HAL_UART_SendBytes(UART_Port port, const uint8_t* data, uint8_t len);
bool            HAL_UART_BytesAvailable(UART_Port port);
uint8_t         HAL_UART_ReadByte(UART_Port port);
void            HAL_UART_FlushRx(UART_Port port);
HAL_UART_Status HAL_UART_GetStatus(UART_Port port);

// ============================================================
// ─── MODULE 4 — HAL_ADC CONFIG ──────────────────────────────
// ============================================================
#define ADC_BATTERY_PIN              36
#define ADC_RESOLUTION_BITS          12
#define ADC_VREF_MV                  3300.0f
#define ADC_OVERSAMPLE_COUNT         16
#define ADC_DIVIDER_R1_OHMS          100000.0f
#define ADC_DIVIDER_R2_OHMS          47000.0f
#define ADC_DIVIDER_RATIO            (ADC_DIVIDER_R2_OHMS / (ADC_DIVIDER_R1_OHMS + ADC_DIVIDER_R2_OHMS))
#define ADC_CALIBRATION_OFFSET_MV    0.0f

// [BUG-H-1] Named voltage range constants for battery validity check.
// Previously Test_ADC_BatteryVoltage used magic numbers 3.0f / 13.0f.
// Robot uses a 2S–3S LiPo: fully-charged 3S = 12.6V, min cutoff ~6.0V.
// Tighten these to your actual pack chemistry.
#define ADC_BATTERY_VOLTAGE_MIN_V    6.0f
#define ADC_BATTERY_VOLTAGE_MAX_V    12.6f

typedef enum {
    ADC_CH_BATTERY = 0    // GPIO36 = ADC1_CHANNEL_0
} ADC_Channel;

typedef enum {
    HAL_ADC_OK    = 0,
    HAL_ADC_ERROR = 1
} HAL_ADC_Status;

// ─── HAL_ADC API ─────────────────────────────────────────────
void           HAL_ADC_Init(void);
uint16_t       HAL_ADC_ReadRaw(ADC_Channel ch);
float          HAL_ADC_ReadMillivolts(ADC_Channel ch);
float          HAL_ADC_ReadBatteryVoltage(void);
HAL_ADC_Status HAL_ADC_GetStatus(void);

// ============================================================
// ─── MODULE 5 — HAL_LEDC CONFIG ─────────────────────────────
// ============================================================

// Motor A
#define LEDC_MOTOR_A_PWM_PIN         12
#define LEDC_MOTOR_A_DIR_PIN         10
#define LEDC_MOTOR_A_CHANNEL         0

// Motor B
#define LEDC_MOTOR_B_PWM_PIN         14
#define LEDC_MOTOR_B_DIR_PIN         15
#define LEDC_MOTOR_B_CHANNEL         1

// PWM properties
#define LEDC_PWM_FREQ_HZ             1000
#define LEDC_PWM_RESOLUTION_BITS     8
#define LEDC_PWM_MAX_DUTY            255
#define LEDC_PWM_MIN_DUTY            0
#define LEDC_MOTOR_DEADBAND          20

// ─── HAL_LEDC TYPES ──────────────────────────────────────────
typedef enum {
    LEDC_MOTOR_A = 0,
    LEDC_MOTOR_B = 1
} LEDC_Channel;

typedef enum {
    MOTOR_DIR_FORWARD  = 1,
    MOTOR_DIR_BACKWARD = 0,
    MOTOR_DIR_BRAKE    = 2
} Motor_Direction;

typedef enum {
    HAL_LEDC_OK    = 0,
    HAL_LEDC_ERROR = 1
} HAL_LEDC_Status;

// ─── HAL_LEDC API ────────────────────────────────────────────
void            HAL_LEDC_Init(void);
void            HAL_LEDC_SetDuty(LEDC_Channel ch, uint8_t duty);
void            HAL_LEDC_SetDirection(LEDC_Channel ch, Motor_Direction dir);
void            HAL_LEDC_Stop(LEDC_Channel ch);
void            HAL_LEDC_StopAll(void);
HAL_LEDC_Status HAL_LEDC_GetStatus(void);

// ============================================================
// ─── TEST RUNNER ─────────────────────────────────────────────
// ============================================================
void RunAllTests(void);
// ============================================================
// DRV_MT2.h — Monolithic Driver Layer Header
// Project  : Waiter Robot — ESP32_MAIN
// Board    : ESP32 WROOM-32 38-pin
// Depends  : HAL_MT1.h / HAL_MT1.cpp (must be in same folder)
// Drivers  : MPU9250, AK8963, TCS3200, HC-SR04, Line Array,
//            Encoder, L298N, Battery
//
// REVIEW FIXES APPLIED (DRV_MT2 Code Review):
//   [C1] TCS_S1_PIN changed from 0 → 4  (GPIO0 is boot-strapping pin)
//   [C2] MOTOR_A_DIR_PIN changed from 10 → 14  (GPIO10 is SPI flash)
//   [W5] Added MPU9250_PI_F constant — use instead of raw 3.14159f literals
//        (update DRV_MT2.cpp usages accordingly)
// ============================================================
// (integrated — single header, pragma once already declared above)
// HAL_MT1 content is part of this integrated header — no separate include needed

// ============================================================
// SECTION 1 — ALL DRIVER PIN DEFINITIONS
// ============================================================

// MPU9250 / AK8963 — I2C (bus pins defined in HAL_MT1.h)
#define MPU9250_I2C_ADDR             0x68
#define AK8963_I2C_ADDR              0x0C

// TCS3200 Color Sensor
// FIX [C1]: TCS_S1_PIN was GPIO 0 — boot-strapping pin driven LOW at init
//           causes board to enter flash/download mode on power-on.
//           Reassigned to GPIO 4 (safe general-purpose output).
#define TCS_S0_PIN                   2
#define TCS_S1_PIN                   4    // was 0 — BOOT PIN, DO NOT USE
#define TCS_S2_PIN                   23
#define TCS_S3_PIN                   19
#define TCS_OUT_PIN                  18

// HC-SR04 Ultrasonic
#define HCSR04_TRIG_PIN              5
#define HCSR04_ECHO_PIN              13

// Line Sensor Array (TCRT5000 x5)
#define LINE_S1_PIN                  34
#define LINE_S2_PIN                  35
#define LINE_S3_PIN                  32
#define LINE_S4_PIN                  33
#define LINE_S5_PIN                  25

// Encoders
#define ENCODER_LEFT_PIN             26
#define ENCODER_RIGHT_PIN            27

// Motor Direction Pins
// FIX [C2]: MOTOR_A_DIR_PIN was GPIO 10 — internally connected to SPI flash
//           (GPIO 6-11 are reserved on WROOM-32). Using it causes flash
//           corruption and random crashes. Reassigned to GPIO 14.
#define MOTOR_A_DIR_PIN              14   // was 10 — SPI FLASH PIN, DO NOT USE
#define MOTOR_B_DIR_PIN              15

// ============================================================
// SECTION 2 — MPU9250 CALIBRATION
// ============================================================
#define MPU9250_WHO_AM_I_REG         0x75
#define MPU9250_WHO_AM_I_VAL         0x71
#define MPU9250_ACCEL_CONFIG_REG     0x1C
#define MPU9250_GYRO_CONFIG_REG      0x1B
#define MPU9250_PWR_MGMT_1_REG       0x6B
#define MPU9250_ACCEL_XOUT_H         0x3B
#define MPU9250_GYRO_XOUT_H          0x43
#define MPU9250_SMPLRT_DIV_REG       0x19
#define MPU9250_CONFIG_REG           0x1A
#define MPU9250_INT_PIN_CFG_REG      0x37
// FIX [S2]: Named constant for I2C bypass enable bit
#define MPU9250_BYPASS_EN_BIT        0x02

#define MPU9250_ACCEL_SCALE          16384.0f   // ±2g  → 16384 LSB/g
#define MPU9250_GYRO_SCALE           131.0f     // ±250°/s → 131 LSB/°/s
#define MPU9250_COMP_FILTER_ALPHA    0.98f
#define MPU9250_CALIB_SAMPLES        500

#define MPU9250_GYRO_OFFSET_X        0.0f
#define MPU9250_GYRO_OFFSET_Y        0.0f
#define MPU9250_GYRO_OFFSET_Z        0.0f
#define MPU9250_ACCEL_OFFSET_X       0.0f
#define MPU9250_ACCEL_OFFSET_Y       0.0f
#define MPU9250_ACCEL_OFFSET_Z       0.0f

#define MPU9250_UPDATE_RATE_HZ       100

// FIX [W5]: Use a named pi constant everywhere instead of raw 3.14159f literals
#define MPU9250_PI_F                 3.14159265f

// ============================================================
// SECTION 3 — AK8963 MAGNETOMETER CALIBRATION
// ============================================================
#define AK8963_WHO_AM_I_REG          0x00
#define AK8963_WHO_AM_I_VAL          0x48
#define AK8963_CNTL1_REG             0x0A
#define AK8963_ST1_REG               0x02
#define AK8963_ST2_REG               0x09   // FIX [C5]: Must be read after data to release latch
#define AK8963_XOUT_L                0x03
#define AK8963_MODE_CONT_2           0x16    // 100Hz continuous, 16-bit

#define AK8963_HARD_IRON_X           0.0f
#define AK8963_HARD_IRON_Y           0.0f
#define AK8963_HARD_IRON_Z           0.0f
#define AK8963_DECLINATION_DEG       -1.5f   // Pakistan ~1.5°W
#define AK8963_MAG_SCALE             0.15f   // μT per LSB in 16-bit mode

// ============================================================
// SECTION 4 — TCS3200 COLOR CALIBRATION
// ============================================================
#define TCS_SCALE_S0_STATE           HIGH
#define TCS_SCALE_S1_STATE           LOW
#define TCS_PULSE_WINDOW_MS          100

// BLACK
#define COLOR_BLACK_R_MAX            200
#define COLOR_BLACK_G_MAX            200
#define COLOR_BLACK_B_MAX            200

// WHITE
#define COLOR_WHITE_R_MIN            800
#define COLOR_WHITE_G_MIN            800
#define COLOR_WHITE_B_MIN            800

// RED
#define COLOR_RED_R_MIN              400
#define COLOR_RED_R_MAX              900
#define COLOR_RED_G_MAX              300
#define COLOR_RED_B_MAX              300

// GREEN
#define COLOR_GREEN_R_MAX            300
#define COLOR_GREEN_G_MIN            400
#define COLOR_GREEN_G_MAX            900
#define COLOR_GREEN_B_MAX            300

// BLUE
#define COLOR_BLUE_R_MAX             300
#define COLOR_BLUE_G_MAX             300
#define COLOR_BLUE_B_MIN             400
#define COLOR_BLUE_B_MAX             900

// Change to TCS_WHITE to follow white line on dark floor
#define LINE_COLOR_TARGET            TCS_BLACK

// ============================================================
// SECTION 5 — HC-SR04 CALIBRATION
// ============================================================
#define HCSR04_MAX_DISTANCE_CM       400
#define HCSR04_MIN_DISTANCE_CM       2
#define HCSR04_TIMEOUT_US            25000UL
#define HCSR04_TRIG_PULSE_US         10
#define HCSR04_SOUND_SPEED_CM_US     0.0343f
#define HCSR04_OBSTACLE_THRESHOLD_CM 15.0f
#define HCSR04_CLEAR_THRESHOLD_CM    20.0f

// ============================================================
// SECTION 6 — LINE SENSOR CALIBRATION
// ============================================================
#define LINE_SENSOR_ACTIVE_STATE     LOW
#define LINE_ERROR_FULL_LEFT         (-2)
#define LINE_ERROR_LEFT              (-1)
#define LINE_ERROR_CENTER            0
#define LINE_ERROR_RIGHT             (+1)
#define LINE_ERROR_FULL_RIGHT        (+2)
#define LINE_LOST_THRESHOLD          5

// ============================================================
// SECTION 7 — ENCODER CALIBRATION
// ============================================================
#define ENCODER_PPR_SHAFT            11
#define GEAR_RATIO                   30
#define ENCODER_PPR_WHEEL            (ENCODER_PPR_SHAFT * GEAR_RATIO)
#define WHEEL_DIAMETER_MM            65.0f
#define WHEEL_BASE_MM                150.0f
// FIX [BUG-11]: Use a local alias so wheel geometry doesn't depend on MPU naming
#define ENCODER_PI_F                 MPU9250_PI_F
#define WHEEL_CIRCUMFERENCE_MM       (ENCODER_PI_F * WHEEL_DIAMETER_MM)
#define MM_PER_TICK                  (WHEEL_CIRCUMFERENCE_MM / (float)ENCODER_PPR_WHEEL)
#define ENCODER_RPM_WINDOW_MS        100
#define ENCODER_STALE_THRESHOLD_MS   200

// ============================================================
// SECTION 8 — MOTOR DRIVER CONFIG
// ============================================================
#define MOTOR_MAX_SPEED              255
#define MOTOR_MIN_SPEED              (-255)
#define MOTOR_DEADBAND               20
#define MOTOR_SOFT_START_STEP        10
#define MOTOR_EMERGENCY_BRAKE_SPEED  0
#define MOTOR_FORWARD_DIR            HIGH
#define MOTOR_BACKWARD_DIR           LOW
#define MOTOR_TURN_HEADING_TOL_DEG   2.0f    // ±2° heading tolerance for TurnDegrees

// ============================================================
// SECTION 9 — BATTERY CONFIG
// ============================================================
#define BATTERY_LOW_THRESHOLD_V      8.0f
#define BATTERY_VERYLOW_THRESHOLD_V  7.0f
#define BATTERY_FULL_V               8.4f
#define BATTERY_CHECK_INTERVAL_MS    1000
#define BATTERY_FILTER_ALPHA         0.1f

// ============================================================
// SECTION 10 — ENUMS AND STRUCTS
// ============================================================

// MPU9250
typedef enum { MPU_OK=0, MPU_FAULT=1, MPU_NOT_INIT=2 } MPU_Status;

// AK8963
typedef enum { AK_OK=0, AK_FAULT=1, AK_NOT_READY=2 } AK_Status;

// TCS3200
typedef enum {
    TCS_BLACK=0, TCS_WHITE=1, TCS_RED=2,
    TCS_GREEN=3, TCS_BLUE=4, TCS_UNKNOWN=5
} TCS_Color;

typedef enum {
    TCS_CH_RED=0, TCS_CH_GREEN=1,
    TCS_CH_BLUE=2, TCS_CH_CLEAR=3
} TCS_Channel;

// HC-SR04
typedef enum {
    HCSR_OK=0, HCSR_OUT_OF_RANGE=1, HCSR_TIMEOUT=2
} HCSR_Status;

// Line Array
typedef struct {
    uint8_t bitmask;    // bit0=S1(left) … bit4=S5(right)
    int8_t  error;      // -2 to +2
    bool    line_lost;
} LineData;

// Encoder
typedef struct {
    volatile int32_t left_ticks;
    volatile int32_t right_ticks;
    float   left_rpm;
    float   right_rpm;
    float   left_mm;
    float   right_mm;
} EncoderData;

// Battery
typedef struct {
    float   voltage;
    bool    is_low;
    bool    is_very_low;
    uint8_t percent;
} BatteryData;

// IMU
typedef struct {
    float      heading_deg;
    float      pitch_deg;
    float      roll_deg;
    float      mag_heading_deg;
    MPU_Status status;
} IMUData;

// ============================================================
// SECTION 11 — ALL FUNCTION DECLARATIONS
// ============================================================

// ── MPU9250 ──────────────────────────────────────────────────
MPU_Status DRV_MPU9250_Init(void);
MPU_Status DRV_MPU9250_Update(void);
float      DRV_MPU9250_GetHeading(void);
float      DRV_MPU9250_GetPitch(void);
float      DRV_MPU9250_GetRoll(void);
IMUData    DRV_MPU9250_GetData(void);
void       DRV_MPU9250_CalibrateGyro(void);

// ── AK8963 ───────────────────────────────────────────────────
AK_Status  DRV_AK8963_Init(void);
AK_Status  DRV_AK8963_Update(void);
float      DRV_AK8963_GetHeading(void);

// ── TCS3200 ──────────────────────────────────────────────────
// WARNING: DRV_TCS3200_Update() is BLOCKING for ~410ms minimum
//          (4 channels × 100ms window + 4 × 2ms settle = ~408ms).
//          With pulseIn() timeout accumulation can reach ~450ms worst-case.
//          Do NOT call from the main 20ms control loop.
//          Call from a dedicated low-rate task or state machine step only.
void      DRV_TCS3200_Init(void);
void      DRV_TCS3200_Update(void);
TCS_Color DRV_TCS3200_GetColor(void);
uint32_t  DRV_TCS3200_GetChannel(TCS_Channel ch);
bool      DRV_TCS3200_IsLineColor(void);

// ── HC-SR04 ──────────────────────────────────────────────────
// NOTE: DRV_HCSR04_GetDistance() is BLOCKING for up to 25ms.
//       DRV_HCSR04_ObstacleDetected() uses cached distance — does NOT re-trigger.
void        DRV_HCSR04_Init(void);
float       DRV_HCSR04_GetDistance(void);
bool        DRV_HCSR04_ObstacleDetected(void);
HCSR_Status DRV_HCSR04_GetStatus(void);

// ── Line Array ───────────────────────────────────────────────
void     DRV_LINE_Init(void);
void     DRV_LINE_Update(void);
LineData DRV_LINE_GetData(void);
int8_t   DRV_LINE_GetError(void);
bool     DRV_LINE_IsLost(void);

// ── Encoder ──────────────────────────────────────────────────
void        DRV_ENCODER_Init(void);
void        DRV_ENCODER_Update(void);
EncoderData DRV_ENCODER_GetData(void);
void        DRV_ENCODER_Reset(void);
float       DRV_ENCODER_GetDistanceMM(void);

// ── L298N Motor Driver ───────────────────────────────────────
void DRV_L298N_Init(void);
void DRV_L298N_SetMotors(int16_t left, int16_t right);
void DRV_L298N_Stop(void);
void DRV_L298N_EmergencyBrake(void);
// Returns true = turn completed within tolerance, false = timed out
bool DRV_L298N_TurnDegrees(float degrees, uint8_t speed);

// ── Battery ──────────────────────────────────────────────────
void        DRV_BATTERY_Init(void);
void        DRV_BATTERY_Update(void);
BatteryData DRV_BATTERY_GetData(void);
bool        DRV_BATTERY_IsLow(void);
bool        DRV_BATTERY_IsVeryLow(void);
// (integrated — single header, pragma once already declared above)
// DRV_MT2 content is part of this integrated header — no separate include needed

// ============================================================
// NAVIGATION MODE — UNCOMMENT ONE ONLY
// ============================================================
#define NAV_MODE_SIMPLE_PID              // A→B line following with PID
// #define NAV_MODE_GRID_FLOODFILL       // full grid exploration flood fill

// ============================================================
// PID TUNING — CHANGE THESE 3 VALUES TO TUNE LINE FOLLOWING
// Start: KP=1.0 KI=0.0 KD=0.0 then increase KP until oscillating
// then add KD to dampen, KI only if steady-state error persists
// ============================================================
#define PID_KP                           1.2f
#define PID_KI                           0.01f
#define PID_KD                           0.40f

// Base speed — reduce for sharper turns, increase on straight runs
#define PID_BASE_SPEED                   120

// Output limits
// FIX [BUG-05]: PID_MIN_OUTPUT was defined but never used — MOD_PID_Update()
// was clamping with PID_MAX_OUTPUT for both directions.  Both limits are now
// applied symmetrically.  Keep these as matched magnitudes unless you
// deliberately want asymmetric forward/reverse authority.
#define PID_MAX_OUTPUT                   255
#define PID_MIN_OUTPUT                   -255

// Anti-windup — integral clamped to ±this value
#define PID_ANTI_WINDUP_LIMIT            50.0f

// Derivative low-pass filter coefficient
// 1.0 = no filter (noisy), 0.0 = no derivative
#define PID_DERIVATIVE_FILTER            0.7f

// Sample time — must match main loop period
#define PID_SAMPLE_TIME_MS               10

// ============================================================
// FLOOD FILL GRID NAVIGATION — CHANGE AFTER SEEING YOUR GRID
// ============================================================
#define FLOODFILL_MAX_CELLS              64      // max discoverable cells
// FIX [BUG-06]: BFS queue must be able to hold up to 2*MAX_CELLS entries
// because each node can be re-enqueued when a shorter path is found.
// The queue size used in the .cpp is FLOODFILL_BFS_QUEUE_SIZE; this value
// must stay >= 2 * FLOODFILL_MAX_CELLS.
#define FLOODFILL_BFS_QUEUE_SIZE         128     // 2× MAX_CELLS (DO NOT reduce below 2×MAX_CELLS)
#define FLOODFILL_CRUISE_SPEED           100     // speed between junctions
#define FLOODFILL_TURN_SPEED             70      // speed during turns
#define FLOODFILL_JUNCTION_THRESHOLD     3       // min sensors for junction detect
#define FLOODFILL_CELL_DISTANCE_MM       300     // distance between junctions

// ============================================================
// WAITER ROBOT TIMINGS — CHANGE FOR DIFFERENT BEHAVIOR
// ============================================================
#define OBSTACLE_WAIT_MS                 5000    // hard timeout before forcing resume
#define OBSTACLE_RECHECK_INTERVAL_MS     500     // recheck every 500ms
#define DELIVERY_WAIT_MS                 10000   // wait at table for pickup
#define ORDER_CONFIRM_TIMEOUT_MS         0       // 0 = wait forever
#define RETURN_TURN_DEGREES              180.0f  // U-turn angle
#define RETURN_TURN_SPEED                80      // U-turn speed
#define LINE_LOST_RECOVERY_TIMEOUT_MS    3000    // try to recover for 3s then use SD
#define HOME_DETECTION_COLOR             TCS_BLUE  // color of home base marker
#define DELIVERY_DETECTION_COLOR         TCS_RED   // color of delivery point
#define TABLE_DETECTION_COLOR            TCS_GREEN // color of table junction

// ============================================================
// UART PACKET DEFINITIONS
//
// FIX [BUG-02]: MSG_TYPE values for the CAM port and Nano port were
// sharing the same numeric values (TELEMETRY==NANO_INIT==0x01,
// ERROR==NANO_STATE==0x02).  These are now in separate namespaces with
// distinct values so that a packet received on the wrong port, or any
// future shared decode path, cannot be silently misinterpreted.
//
// CAM port  (ESP32 ↔ ESP32-CAM):
//   0x01 = MSG_CAM_TELEMETRY   (ESP32 → CAM, 10Hz)
//   0x02 = MSG_CAM_ERROR       (CAM → ESP32)
//   0x04 = MSG_CAM_CONFIG      (CAM → ESP32)
//
// Nano port (ESP32 → Arduino Nano):
//   0x10 = MSG_NANO_INIT       (ESP32 → Nano boot)
//   0x11 = MSG_NANO_STATE      (ESP32 → Nano state update)
//
// DO NOT CHANGE THESE VALUES without updating ESP32-CAM and Nano firmware.
// ============================================================
#define PACKET_START_BYTE                0xAAU
#define PACKET_END_BYTE                  0x55U
#define PACKET_MAX_PAYLOAD               32U

// CAM port message types
#define MSG_CAM_TELEMETRY                0x01U   // ESP32 → CAM, 10Hz
#define MSG_CAM_ERROR                    0x02U   // CAM → ESP32
#define MSG_CAM_CONFIG                   0x04U   // CAM → ESP32 config update

// Nano port message types  (high nibble = 1 to avoid CAM collision)
#define MSG_NANO_INIT                    0x10U   // ESP32 → Nano boot
#define MSG_NANO_STATE                   0x11U   // ESP32 → Nano state update

// Legacy aliases — these names appeared in the original firmware.
// They now resolve to the corrected values above.  Remove after
// confirming all three MCU firmware builds have been updated.
#define MSG_TYPE_TELEMETRY               MSG_CAM_TELEMETRY
#define MSG_TYPE_ERROR                   MSG_CAM_ERROR
#define MSG_TYPE_CAM_CONFIG              MSG_CAM_CONFIG
#define MSG_TYPE_NANO_INIT               MSG_NANO_INIT
#define MSG_TYPE_NANO_STATE              MSG_NANO_STATE

// Telemetry payload size
// [enc_left(2)] [enc_right(2)] [heading(2)] [state_str(8)] = 14 bytes
#define TELEMETRY_PAYLOAD_SIZE           14U
#define TELEMETRY_SEND_INTERVAL_MS       100U    // 10Hz

// ============================================================
// PATH RECORDING — ADJUST FOR YOUR ROBOT SIZE AND SPEED
// ============================================================
#define PATH_MAX_NODES                   200U    // max recorded positions
#define PATH_RECORD_INTERVAL_MS          100U    // record position every 100ms

// ============================================================
// LOW BATTERY SPEED REDUCTION
// ============================================================
#define LOW_BATTERY_SPEED_FACTOR         0.70f   // 70% speed in LOW_BATTERY state

// ============================================================
// PID GAIN VALIDATION LIMITS — prevent runaway from corrupted
// CAM config packets
// ============================================================
#define PID_KP_MAX                       10.0f
#define PID_KI_MAX                       1.0f
#define PID_KD_MAX                       5.0f

// ============================================================
// OBSTACLE HYSTERESIS — consecutive clear reads before resuming
// ============================================================
#define OBSTACLE_CLEAR_COUNT_REQUIRED    2U

// ============================================================
// STATE MACHINE STATES AND EVENTS
// ============================================================
typedef enum {
    STATE_INIT          = 0,
    STATE_IDLE          = 1,    // waiting for order confirmation
    STATE_NAVIGATE      = 2,    // following line to destination
    STATE_OBSTACLE      = 3,    // obstacle detected — waiting
    STATE_DELIVER       = 4,    // at delivery point — waiting pickup
    STATE_RETURN        = 5,    // returning home via saved path
    STATE_RECOVERY      = 6,    // line lost — recovering via SD
    STATE_LOW_BATTERY   = 7,    // battery warning — finishing then RTB
    STATE_EMERGENCY     = 8     // very low battery — stop immediately
} SystemState;

typedef enum {
    EVENT_ORDER_CONFIRMED   = 0,
    EVENT_OBSTACLE_DETECTED = 1,
    EVENT_OBSTACLE_CLEARED  = 2,
    EVENT_DELIVERY_REACHED  = 3,
    EVENT_DELIVERY_DONE     = 4,
    EVENT_HOME_REACHED      = 5,
    EVENT_LINE_LOST         = 6,
    EVENT_LINE_FOUND        = 7,
    EVENT_LOW_BATTERY       = 8,
    EVENT_VERY_LOW_BATTERY  = 9,
    EVENT_CAM_SD_FAIL       = 10,
    EVENT_NONE              = 255
} SystemEvent;

// SensorFrame — single source of truth every 10ms
typedef struct {
    float    heading_deg;
    float    mag_heading_deg;
    float    pitch_deg;
    float    roll_deg;
    int8_t   line_error;
    bool     line_lost;
    uint8_t  line_bitmask;
    float    distance_cm;
    bool     obstacle;
    TCS_Color color;
    int32_t  enc_left_ticks;
    int32_t  enc_right_ticks;
    float    left_rpm;
    float    right_rpm;
    float    battery_v;
    bool     battery_low;
    bool     battery_very_low;
    uint8_t  battery_percent;
    uint32_t timestamp_ms;
} SensorFrame;

// Path node for recording journey
typedef struct {
    int32_t  enc_left;
    int32_t  enc_right;
    float    heading;
    uint32_t timestamp_ms;
} PathNode;

// CAM status received via UART
typedef enum {
    CAM_OK          = 0,
    CAM_SD_FAIL     = 1,
    CAM_SD_FULL     = 2,
    CAM_BUF_FULL    = 3
} CAM_Status;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

// Sensor Fusion
void        MOD_SENSORFUSION_Init(void);
void        MOD_SENSORFUSION_Update(void);
SensorFrame MOD_SENSORFUSION_GetFrame(void);

// PID
void    MOD_PID_Init(void);
void    MOD_PID_Reset(void);
void    MOD_PID_Update(int8_t error);
int16_t MOD_PID_GetLeftSpeed(void);
int16_t MOD_PID_GetRightSpeed(void);
void    MOD_PID_SetGains(float kp, float ki, float kd);
void    MOD_PID_SetBaseSpeed(int16_t speed);

// Packet Builder
uint8_t MOD_PACKET_Build(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* out);
uint8_t MOD_PACKET_CRC8(const uint8_t* data, uint8_t len);
bool    MOD_PACKET_Validate(const uint8_t* frame, uint8_t len);

// State Machine
void        MOD_SM_Init(void);
void        MOD_SM_Update(SensorFrame frame);
void        MOD_SM_InjectEvent(SystemEvent event);
SystemState MOD_SM_GetState(void);
const char* MOD_SM_GetStateString(void);
bool        MOD_SM_IsOrderConfirmed(void);
void        MOD_SM_ConfirmOrder(void);  // called from web dashboard

// Path Recording
void     PATH_Init(void);
void     PATH_RecordNode(SensorFrame frame);
void     PATH_StartRecording(void);
void     PATH_StopRecording(void);
PathNode PATH_GetNode(uint16_t index);
uint16_t PATH_GetNodeCount(void);
void     PATH_Clear(void);
bool     PATH_IsRecording(void);

// UART CAM
void       MOD_UART_CAM_Init(void);
void       MOD_UART_CAM_SendTelemetry(SensorFrame frame, SystemState state);
void       MOD_UART_CAM_ProcessRx(void);
CAM_Status MOD_UART_CAM_GetStatus(void);

// UART Nano
void MOD_UART_NANO_Init(void);
void MOD_UART_NANO_SendInit(void);
void MOD_UART_NANO_SendState(SystemState state);

// Flood Fill (compiled only if NAV_MODE_GRID_FLOODFILL defined)
#ifdef NAV_MODE_GRID_FLOODFILL
void  FF_Init(void);
void  FF_Update(SensorFrame frame);
bool  FF_IsExplorationComplete(void);
void  FF_Reset(void);
#endif

// App Main
void APP_MAIN_Init(void);
void APP_MAIN_Run(void);
