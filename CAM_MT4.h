#pragma once
#include <Arduino.h>
#include <SD_MMC.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "esp_camera.h"
#include "SPIFFS.h"

// _sd_mounted is defined in the .cpp; used by web handlers for SD guard
extern bool _sd_mounted;

// ============================================================
// ESP32-CAM PIN DEFINITIONS — AI-THINKER BOARD
// ============================================================
#define CAM_UART_RX_PIN              3     // U0RXD — from ESP32
#define CAM_UART_TX_PIN              1     // U0TXD — to ESP32
#define CAM_UART_BAUD                115200

// SD Card — 1-bit SPI mode (via SD_MMC)
#define SD_CLK_PIN                   14
#define SD_CMD_PIN                   15    // MOSI
#define SD_DATA0_PIN                 2     // MISO — has 10k pullup
#define SD_CS_PIN                    13

// OV2640 Camera Pin Map — AI-Thinker ESP32-CAM
#define CAM_PIN_PWDN                 32
#define CAM_PIN_RESET                -1
#define CAM_PIN_XCLK                 0
#define CAM_PIN_SIOD                 26
#define CAM_PIN_SIOC                 27
#define CAM_PIN_D7                   35
#define CAM_PIN_D6                   34
#define CAM_PIN_D5                   39
#define CAM_PIN_D4                   36
#define CAM_PIN_D3                   21
#define CAM_PIN_D2                   19
#define CAM_PIN_D1                   18
#define CAM_PIN_D0                   5
#define CAM_PIN_VSYNC                25
#define CAM_PIN_HREF                 23
#define CAM_PIN_PCLK                 22

// ============================================================
// WIFI CREDENTIALS — LOADED FROM NVS AT RUNTIME
// DO NOT hardcode credentials here. Use CONFIG_ProvisionCredentials()
// to write credentials to NVS during first-time setup.
// ============================================================
#define WIFI_CONNECT_TIMEOUT_MS      15000

// ============================================================
// WEB DASHBOARD SECURITY — LOADED FROM NVS AT RUNTIME
// DO NOT hardcode credentials here.
// ============================================================
#define WEB_SERVER_PORT              80
#define WEB_STREAM_PORT              81

// ============================================================
// SD LOGGING CONFIG
// ============================================================
#define SD_LOG_DIR                   "/logs"
#define SD_LOG_MAX_SIZE_MB           500

// Write buffer size must be an integer multiple of LogRecord (20 bytes).
// 500 bytes = 25 records per flush — chosen to fit cleanly.
#define SD_WRITE_BUFFER_SIZE         500    // 25 × 20-byte LogRecords
#define SD_FLUSH_INTERVAL_MS         1000   // flush every 1s
#define SD_MIN_FREE_KB               1024   // 1MB minimum free space

// ============================================================
// RING BUFFER CONFIG — USES PSRAM
// NOTE: This ring buffer is for UART telemetry frames ONLY.
//       Do NOT use it for camera/JPEG image data.
// ============================================================
#define RINGBUF_CAPACITY_BYTES       65536  // 64KB in PSRAM
#define RINGBUF_NEAR_FULL_PERCENT    80     // warn at 80%
#define RINGBUF_FRAME_MAX_SIZE       64     // max single UART telemetry frame bytes

// ============================================================
// WIFI STREAM CONFIG
// ============================================================
#define WIFI_STREAM_ENABLED          1      // 0 to disable
#define STREAM_FRAME_SIZE            FRAMESIZE_QVGA  // 320×240
#define STREAM_QUALITY               10     // 0=best 63=worst JPEG
#define STREAM_FPS_TARGET            10     // target frames/sec

// ============================================================
// ERROR REPORTING CONFIG
// ============================================================
#define ERR_FRAME_INTERVAL_MS        500    // min ms between error reports

// ============================================================
// SECTION 2 — PACKET DEFINITIONS (MUST MATCH ESP32)
// CRC-8/SMBUS — polynomial 0x07, init 0x00, no input/output reflection.
// Both sides MUST use this exact variant.
// ============================================================
#define PACKET_START_BYTE            0xAA
#define PACKET_END_BYTE              0x55
#define MSG_TYPE_TELEMETRY           0x01
#define MSG_TYPE_ERROR               0x02
#define MSG_TYPE_CAM_CONFIG          0x04

// Error codes sent to ESP32
#define ERR_CODE_SD_FAIL             0x01
#define ERR_CODE_SD_FULL             0x02
#define ERR_CODE_BUF_FULL            0x03

// ============================================================
// SECTION 3 — TYPES AND STRUCTS
// ============================================================
typedef enum {
    SD_OK=0, SD_WRITE_FAIL=1, SD_FULL=2, SD_NOT_MOUNTED=3
} SD_Status;

typedef enum {
    CAM_ERR_NONE=0, CAM_ERR_SD_FAIL=1,
    CAM_ERR_SD_FULL=2, CAM_ERR_BUF_FULL=3
} CAM_ErrorCode;

// Parsed telemetry frame from ESP32.
// heading_x10 valid range: 0–3599 (represents 0.0°–359.9°). Negative values are invalid.
// state_str: 8 printable chars + null terminator. Always null-terminate before use.
typedef struct {
    int16_t  enc_left;
    int16_t  enc_right;
    int16_t  heading_x10;    // heading * 10 for integer transfer; range [0, 3599]
    char     state_str[9];   // 8 chars + null terminator
    uint32_t timestamp_ms;
    bool     valid;
} TelemetryFrame;

// SD binary log record — 20 bytes fixed.
// state_str is NOT null-terminated in this struct (binary log, not C-string).
// Always use memcpy with explicit length when reading/writing state_str.
// Do NOT pass LogRecord.state_str directly to string functions (printf, strcmp, etc.).
typedef struct {
    uint32_t timestamp_ms;
    int16_t  enc_left;
    int16_t  enc_right;
    int16_t  heading_x10;
    char     state_str[8];   // NOTE: not null-terminated — binary field only
} __attribute__((packed)) LogRecord;

// Runtime config — saved to SPIFFS, changeable from web dashboard.
// Integrity is verified by CRC32 over all fields except the crc32 field itself.
// WiFi and web credentials are stored separately in NVS, not in this struct.
typedef struct {
    float    pid_kp;
    float    pid_ki;
    float    pid_kd;
    int16_t  base_speed;
    uint8_t  line_color_target;    // 0=black 1=white
    uint8_t  nav_mode;             // 0=simple 1=grid
    float    battery_low_v;
    float    battery_verylow_v;
    uint32_t crc32;               // CRC32 over all fields above; computed on save, verified on load
} RobotConfig;

// ============================================================
// SECTION 4 — FUNCTION DECLARATIONS
// ============================================================

// HAL UART CAM
void    HAL_UART_CAM_Init(void);
void    HAL_UART_CAM_SendBytes(const uint8_t* data, uint16_t len);
bool    HAL_UART_CAM_Available(void);
uint8_t HAL_UART_CAM_ReadByte(void);

// HAL SD
SD_Status HAL_SD_Init(void);
// BUG FIX [B1]: const qualifier added — HAL_SD_WriteRecord must not modify the record.
// Original declaration took a non-const pointer with no justification.
SD_Status HAL_SD_WriteRecord(const LogRecord* rec);
SD_Status HAL_SD_Flush(void);
uint32_t  HAL_SD_GetFreeKB(void);
bool      HAL_SD_CreateSession(uint32_t session_id);
void      HAL_SD_CloseSession(void);

// Ring Buffer
void     RINGBUF_Init(void);
// BUG FIX [B2]: const qualifier added — Push must not modify the frame data.
bool     RINGBUF_Push(const uint8_t* frame, uint16_t len);
bool     RINGBUF_Pop(uint8_t* out, uint16_t* out_len);
bool     RINGBUF_IsEmpty(void);
bool     RINGBUF_IsNearFull(void);
void     RINGBUF_Flush(void);
uint16_t RINGBUF_GetCount(void);

// Frame Parser
void           FRAMEPARSER_Init(void);
void           FRAMEPARSER_Update(void);
TelemetryFrame FRAMEPARSER_GetLatest(void);
uint32_t       FRAMEPARSER_GetCorruptCount(void);

// SD Logger
void      SDLOGGER_Init(void);
void      SDLOGGER_Update(void);
uint32_t  SDLOGGER_GetFrameCount(void);
SD_Status SDLOGGER_GetStatus(void);

// Error Reporter
void ERRREPORTER_Init(void);
void ERRREPORTER_Report(CAM_ErrorCode code);

// WiFi Stream
bool WIFISTREAM_Init(void);
void WIFISTREAM_StartStream(void);
bool WIFISTREAM_IsActive(void);

// Web Dashboard
void WEBDASH_Init(void);
void WEBDASH_Handle(void);
void WEBDASH_SendConfigToESP32(const RobotConfig* cfg);

// Config SPIFFS
// Credentials (WiFi SSID/pass, web user/pass) are stored in NVS, not SPIFFS.
// Use CONFIG_ProvisionCredentials() for first-time setup.
void        CONFIG_Init(void);
RobotConfig CONFIG_Load(void);
void        CONFIG_Save(const RobotConfig* cfg);
RobotConfig CONFIG_GetDefaults(void);
// Provision WiFi and web credentials to NVS (call once during setup/factory reset).
void        CONFIG_ProvisionCredentials(const char* wifi_ssid,
                                        const char* wifi_pass,
                                        const char* web_user,
                                        const char* web_pass);

// Packet Builder
// CRC-8/SMBUS — polynomial 0x07, init 0x00, no reflect.
// Must match ESP32 implementation exactly.
uint8_t CAM_PACKET_Build(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* out);
uint8_t CAM_PACKET_CRC8(const uint8_t* data, uint8_t len);
bool    CAM_PACKET_Validate(const uint8_t* frame, uint8_t len);

// App Main CAM
void APP_CAM_Init(void);
// APP_CAM_Run() must be called from Arduino loop() or a dedicated FreeRTOS task.
// Do NOT call from an ISR.
void APP_CAM_Run(void);

// Camera init helper
bool camera_init(void);
