#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_INA219.h>

// ============================================================
// HARDWARE PINS — CHANGE IF WIRING CHANGES
// ============================================================

// UART from ESP32 — hardware UART D0
// DO NOT use D0 during upload — disconnect wire first
#define NANO_UART_BAUD               115200

// I2C — hardware pins A4(SDA) A5(SCL) — do not change
#define INA219_I2C_ADDR              0x40
#define OLED_I2C_ADDR                0x3C

// ============================================================
// OLED DISPLAY CONFIG — SSD1306 128x64
// ============================================================
#define OLED_SCREEN_WIDTH            128
#define OLED_SCREEN_HEIGHT           64
#define OLED_RESET_PIN               -1    // no reset pin
#define OLED_TEXT_SIZE_SMALL         1
#define OLED_TEXT_SIZE_LARGE         2

// ============================================================
// INA219 CALIBRATION
// Calibration: setCalibration_32V_2A() — max 2A, 32V bus.
// If system draws >2A peak, switch to setCalibration_32V_3A2()
// in DRV_INA219_Init() and update this comment.
// ============================================================
#define INA219_SHUNT_OHMS            0.1f   // standard INA219 module shunt
#define INA219_UPDATE_INTERVAL_MS    500    // read every 500ms

// ============================================================
// DISPLAY REFRESH
// ============================================================
#define DISPLAY_POWER_REFRESH_MS     500    // Row 0 refresh rate
#define DISPLAY_BATTERY_BAR_CHARS    10     // chars for battery bar

// ============================================================
// BATTERY DISPLAY THRESHOLDS — MATCH ESP32 CONFIG
// ============================================================
#define DISP_BATTERY_FULL_V          8.4f
#define DISP_BATTERY_LOW_V           8.0f
#define DISP_BATTERY_VERYLOW_V       7.0f

// ============================================================
// PACKET DEFINITIONS — MUST MATCH ESP32
// ============================================================
#define PKT_START_BYTE               0xAA
#define PKT_END_BYTE                 0x55
#define MSG_TYPE_NANO_INIT           0x01
#define MSG_TYPE_NANO_STATE          0x02

// ============================================================
// PROTOCOL NOTE — MSG_TYPE_NANO_INIT PAYLOAD
// The parser rejects payload_len == 0 (resets state machine).
// INIT frames MUST be sent with exactly 1-byte dummy payload (0x00).
// The payload value is ignored; only msg_type matters.
// Document this constraint in the ESP32 sender implementation.
// ============================================================

// ============================================================
// TYPES
// ============================================================

// state_str: up to 8 printable characters + null terminator.
// ALL strings written to this field MUST be <= 8 chars.
typedef struct {
    uint8_t  msg_type;
    char     state_str[9];    // 8 chars + null — DO NOT exceed 8 chars
    bool     valid;
} NanoFrame;

typedef struct {
    float   voltage_v;
    float   current_ma;
    float   power_mw;
    uint8_t battery_percent;
} PowerData;

typedef enum {
    NANO_OK=0, NANO_UART_ERROR=1, NANO_INA_ERROR=2, NANO_OLED_ERROR=3
} Nano_Status;

// ============================================================
// FUNCTION DECLARATIONS — PRODUCTION API
// ============================================================

// HAL UART Nano
void    HAL_UART_NANO_Init(void);
bool    HAL_UART_NANO_Available(void);
uint8_t HAL_UART_NANO_ReadByte(void);
// NEVER add a transmit function — Nano never transmits

// INA219 Driver
bool      DRV_INA219_Init(void);
void      DRV_INA219_Update(void);
PowerData DRV_INA219_GetData(void);

// OLED Driver
void DRV_OLED_Init(void);
void DRV_OLED_Clear(void);
void DRV_OLED_DisplayPower(PowerData data);
void DRV_OLED_DisplayState(const char* state_str);
void DRV_OLED_DisplayBatteryBar(float voltage);
void DRV_OLED_DisplayBootScreen(void);
void DRV_OLED_Update(void);

// Frame Parser Nano
void      FRAMEPARSER_NANO_Init(void);
void      FRAMEPARSER_NANO_Update(void);
NanoFrame FRAMEPARSER_NANO_GetLatest(void);

// App Main Nano
void APP_NANO_Init(void);
void APP_NANO_Run(void);

// ============================================================
// TEST HOOKS — compiled ONLY when TEST_BUILD is defined.
// To enable: add  #define TEST_BUILD  at the top of the .ino
// before #include "NANO_MT5.h".
// NEVER define TEST_BUILD in production firmware.
// ============================================================
#ifdef TEST_BUILD
uint8_t TEST_crc8(const uint8_t* data, uint8_t len);
void    TEST_InjectByte(uint8_t b);
void    TEST_SetLastPowerUpdate(uint32_t t);
int     TEST_FreeRAM(void);
#endif
