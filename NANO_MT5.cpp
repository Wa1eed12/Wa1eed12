#include "NANO_MT5.h"

// ============================================================
// STATIC INSTANCES
// ============================================================
static Adafruit_SSD1306 _display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT,
                                  &Wire, OLED_RESET_PIN);
static Adafruit_INA219  _ina219(INA219_I2C_ADDR);
static PowerData        _power_data        = {0.0f, 0.0f, 0.0f, 0};
static NanoFrame        _latest_frame      = {0, "UNKNOWN", false};
static uint32_t         _last_power_update = 0;

// ============================================================
// CRC8 — polynomial 0x07 — must match ESP32
// ============================================================
static uint8_t _crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ============================================================
// HAL UART NANO
// ============================================================

// ---- Test injection queue (TEST_BUILD only) ---------------
#ifdef TEST_BUILD
#define _TEST_INJECT_BUF_SIZE 64
static uint8_t  _test_inject_buf[_TEST_INJECT_BUF_SIZE];
static uint8_t  _test_inject_head = 0;
static uint8_t  _test_inject_tail = 0;
static uint8_t  _test_inject_count = 0;

// Called by test harness to push bytes into the parser without Serial
void TEST_InjectByte(uint8_t b) {
    if (_test_inject_count < _TEST_INJECT_BUF_SIZE) {
        _test_inject_buf[_test_inject_tail] = b;
        _test_inject_tail = (_test_inject_tail + 1) % _TEST_INJECT_BUF_SIZE;
        _test_inject_count++;
    }
    // If buffer full, byte is silently dropped (test design error)
}

static bool _test_inject_available(void) {
    return _test_inject_count > 0;
}

static uint8_t _test_inject_read(void) {
    uint8_t b = _test_inject_buf[_test_inject_head];
    _test_inject_head = (_test_inject_head + 1) % _TEST_INJECT_BUF_SIZE;
    _test_inject_count--;
    return b;
}
#endif // TEST_BUILD
// -----------------------------------------------------------

void HAL_UART_NANO_Init(void) {
    Serial.begin(NANO_UART_BAUD);
#ifdef TEST_BUILD
    _test_inject_head  = 0;
    _test_inject_tail  = 0;
    _test_inject_count = 0;
#endif
}

bool HAL_UART_NANO_Available(void) {
#ifdef TEST_BUILD
    return _test_inject_available();
#else
    return Serial.available() > 0;
#endif
}

uint8_t HAL_UART_NANO_ReadByte(void) {
#ifdef TEST_BUILD
    return _test_inject_read();
#else
    return (uint8_t)Serial.read();
#endif
}
// No transmit function — enforced by design

// ============================================================
// INA219 DRIVER
// ============================================================

bool DRV_INA219_Init(void) {
    // NOTE: Wire.begin() is called once in APP_NANO_Init() before this.
    // Do NOT call Wire.begin() here again.
    if (!_ina219.begin()) {
        return false;
    }
    // Calibration: 32V bus, max 2A.
    // If system draws >2A peak, switch to setCalibration_32V_3A2().
    _ina219.setCalibration_32V_2A();
    return true;
}

void DRV_INA219_Update(void) {
    float v = _ina219.getBusVoltage_V();
    float i = _ina219.getCurrent_mA();

    // Clamp current BEFORE computing power — power_mw is always
    // the product of the actual output values, never inconsistent.
    if (i < 0.0f) i = 0.0f;
    float p = v * i;
    if (p < 0.0f) p = 0.0f;  // guard against float edge cases

    float pct_raw = ((v - DISP_BATTERY_VERYLOW_V) /
                     (DISP_BATTERY_FULL_V - DISP_BATTERY_VERYLOW_V)) * 100.0f;

    _power_data.voltage_v       = v;
    _power_data.current_ma      = i;
    _power_data.power_mw        = p;
    _power_data.battery_percent = (uint8_t)constrain((int)pct_raw, 0, 100);
}

PowerData DRV_INA219_GetData(void) {
    return _power_data;
}

// ============================================================
// OLED DRIVER
// ============================================================

void DRV_OLED_Init(void) {
    if (!_display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
        // OLED not detected — system continues, display stays blank.
        // No Serial.print here (conflicts with ESP32 UART in production).
        return;
    }
    _display.clearDisplay();
    _display.setTextColor(SSD1306_WHITE);
    DRV_OLED_DisplayBootScreen();
}

void DRV_OLED_Clear(void) {
    _display.clearDisplay();
}

void DRV_OLED_DisplayBootScreen(void) {
    // BLOCKS FOR 2000ms — intentional at boot only.
    // ESP32 MUST delay first UART transmission by >= 2500ms
    // to avoid filling the 64-byte Serial RX buffer during this window.
    _display.clearDisplay();
    _display.setTextColor(SSD1306_WHITE);

    // Size 1: 6px/char wide, 8px tall
    // "WAITER ROBOT" = 12 chars x 6px = 72px → center: (128-72)/2 = 28px
    _display.setTextSize(OLED_TEXT_SIZE_SMALL);

    _display.setCursor(28, 20);
    _display.print(F("WAITER ROBOT"));

    _display.setCursor(28, 36);
    _display.print(F("SYSTEM READY"));

    _display.display();
    delay(2000);
    _display.clearDisplay();
    _display.display();
}

void DRV_OLED_DisplayPower(PowerData data) {
    char buf_v[8];
    char buf_i[10];   // 10 bytes: safe margin for dtostrf width=5 with sign
    char buf_p[8];

    dtostrf(data.voltage_v,          4, 1, buf_v);   // e.g. " 8.2"
    dtostrf(data.current_ma,         5, 0, buf_i);   // e.g. "  450"
    dtostrf(data.power_mw / 1000.0f, 4, 2, buf_p);  // mW -> W, e.g. " 3.69"

    // ---- Row 0 — y=0: "V:X.XV  I:XXXmA" ----
    _display.fillRect(0, 0, OLED_SCREEN_WIDTH, 8, SSD1306_BLACK);
    _display.setTextSize(OLED_TEXT_SIZE_SMALL);
    _display.setTextColor(SSD1306_WHITE);
    _display.setCursor(0, 0);
    _display.print(F("V:"));
    _display.print(buf_v);
    _display.print(F("V  I:"));
    _display.print(buf_i);
    _display.print(F("mA"));

    // ---- Row 1 — y=16: "P: X.XXW" ----
    _display.fillRect(0, 16, OLED_SCREEN_WIDTH, 8, SSD1306_BLACK);
    _display.setCursor(0, 16);
    _display.print(F("P: "));
    _display.print(buf_p);
    _display.print(F("W"));
}

void DRV_OLED_DisplayState(const char* state_str) {
    // ---- Row 2 — y=32: "STATE: XXXXXXXX" ----
    // state_str MUST be <= 8 chars. NanoFrame.state_str enforces this.
    _display.fillRect(0, 32, OLED_SCREEN_WIDTH, 8, SSD1306_BLACK);
    _display.setTextSize(OLED_TEXT_SIZE_SMALL);
    _display.setTextColor(SSD1306_WHITE);
    _display.setCursor(0, 32);
    _display.print(F("STATE: "));
    _display.print(state_str);
}

void DRV_OLED_DisplayBatteryBar(float voltage) {
    static bool     _blink_state   = false;
    static uint32_t _blink_last_ms = 0;

    // Clear row 3
    _display.fillRect(0, 48, OLED_SCREEN_WIDTH, 8, SSD1306_BLACK);
    _display.setTextSize(OLED_TEXT_SIZE_SMALL);
    _display.setTextColor(SSD1306_WHITE);
    _display.setCursor(0, 48);

    // Blinking "!!!LOW BATTERY!!!" for voltage below VERYLOW threshold
    if (voltage < DISP_BATTERY_VERYLOW_V) {
        uint32_t now = millis();
        if (now - _blink_last_ms >= 500) {
            _blink_state   = !_blink_state;
            _blink_last_ms = now;
        }
        if (_blink_state) {
            _display.print(F("!!!LOW BATTERY!!!"));
        }
        return;
    }

    // Compute fill level
    float pct_raw = ((voltage - DISP_BATTERY_VERYLOW_V) /
                     (DISP_BATTERY_FULL_V - DISP_BATTERY_VERYLOW_V)) * 100.0f;
    uint8_t pct    = (uint8_t)constrain((int)pct_raw, 0, 100);
    uint8_t filled = (uint8_t)((pct * DISPLAY_BATTERY_BAR_CHARS) / 100);
    uint8_t empty  = DISPLAY_BATTERY_BAR_CHARS - filled;

    // Blink entire bar row when in LOW range (above VERYLOW, below LOW)
    if (voltage < DISP_BATTERY_LOW_V) {
        uint32_t now = millis();
        if (now - _blink_last_ms >= 500) {
            _blink_state   = !_blink_state;
            _blink_last_ms = now;
        }
        if (!_blink_state) {
            return;
        }
    }

    // Draw bar: [ + '#' * filled + '.' * empty + ] + pct%
    _display.print('[');
    for (uint8_t i = 0; i < filled; i++) _display.print('#');
    for (uint8_t i = 0; i < empty;  i++) _display.print('.');
    _display.print(F("] "));

    char pct_buf[5];
    dtostrf((float)pct, 3, 0, pct_buf);
    _display.print(pct_buf);
    _display.print('%');
}

void DRV_OLED_Update(void) {
    _display.display();
}

// ============================================================
// FRAME PARSER NANO
// ============================================================

typedef enum {
    PSTATE_WAIT_START   = 0,
    PSTATE_WAIT_TYPE    = 1,
    PSTATE_WAIT_LEN     = 2,
    PSTATE_READ_PAYLOAD = 3,
    PSTATE_WAIT_CRC     = 4,
    PSTATE_WAIT_END     = 5
} ParserState;

#define MAX_PAYLOAD_LEN  32

static ParserState  _pstate        = PSTATE_WAIT_START;
static uint8_t      _msg_type      = 0;
static uint8_t      _payload_len   = 0;
static uint8_t      _payload_idx   = 0;
static uint8_t      _payload_buf[MAX_PAYLOAD_LEN];
static uint8_t      _rx_crc        = 0;

void FRAMEPARSER_NANO_Init(void) {
    _pstate      = PSTATE_WAIT_START;
    _msg_type    = 0;
    _payload_len = 0;
    _payload_idx = 0;
    _rx_crc      = 0;
    _latest_frame.valid = false;
    strncpy(_latest_frame.state_str, "UNKNOWN",
            sizeof(_latest_frame.state_str) - 1);
    _latest_frame.state_str[sizeof(_latest_frame.state_str) - 1] = '\0';
}

void FRAMEPARSER_NANO_Update(void) {
    while (HAL_UART_NANO_Available()) {
        // 'rx_byte' avoids shadowing the Arduino 'byte' typedef
        uint8_t rx_byte = HAL_UART_NANO_ReadByte();

        switch (_pstate) {

            case PSTATE_WAIT_START:
                if (rx_byte == PKT_START_BYTE) {
                    _pstate = PSTATE_WAIT_TYPE;
                }
                break;

            case PSTATE_WAIT_TYPE:
                _msg_type = rx_byte;
                _pstate   = PSTATE_WAIT_LEN;
                break;

            case PSTATE_WAIT_LEN:
                if (rx_byte == 0 || rx_byte > MAX_PAYLOAD_LEN) {
                    // Invalid length — reset state machine
                    _pstate = PSTATE_WAIT_START;
                } else {
                    _payload_len = rx_byte;
                    _payload_idx = 0;
                    _pstate      = PSTATE_READ_PAYLOAD;
                }
                break;

            case PSTATE_READ_PAYLOAD:
                _payload_buf[_payload_idx++] = rx_byte;
                if (_payload_idx >= _payload_len) {
                    _pstate = PSTATE_WAIT_CRC;
                }
                break;

            case PSTATE_WAIT_CRC:
                _rx_crc = rx_byte;
                _pstate = PSTATE_WAIT_END;
                break;

            case PSTATE_WAIT_END:
                if (rx_byte == PKT_END_BYTE) {
                    // Validate CRC over [msg_type, payload_len, payload...]
                    uint8_t crc_buf[2 + MAX_PAYLOAD_LEN];
                    crc_buf[0] = _msg_type;
                    crc_buf[1] = _payload_len;
                    for (uint8_t i = 0; i < _payload_len; i++) {
                        crc_buf[2 + i] = _payload_buf[i];
                    }
                    uint8_t calc_crc = _crc8(crc_buf, 2 + _payload_len);

                    if (calc_crc == _rx_crc) {

                        if (_msg_type == MSG_TYPE_NANO_INIT) {
                            // INIT accepted with any non-zero payload length.
                            // Protocol mandates 1-byte dummy payload (see header note).
                            _latest_frame.msg_type = _msg_type;
                            _latest_frame.valid    = true;
                            strncpy(_latest_frame.state_str, "SYS RDY",
                                    sizeof(_latest_frame.state_str) - 1);
                            _latest_frame.state_str[sizeof(_latest_frame.state_str) - 1] = '\0';
                            DRV_OLED_DisplayState(_latest_frame.state_str);
                            // Do NOT call DRV_OLED_Update() here
                            // — power loop handles display flush

                        } else if (_msg_type == MSG_TYPE_NANO_STATE) {
                            // Payload is state string, up to 8 chars
                            uint8_t copy_len = (_payload_len < 8u) ? _payload_len : 8u;
                            memcpy(_latest_frame.state_str, _payload_buf, copy_len);
                            _latest_frame.state_str[copy_len] = '\0';
                            _latest_frame.msg_type = _msg_type;
                            _latest_frame.valid    = true;
                            DRV_OLED_DisplayState(_latest_frame.state_str);
                            // Flush display immediately for state change
                            DRV_OLED_Update();
                        }
                        // Unknown type — silently discard, hold last valid display
                    }
                    // Corrupt CRC — silently discard, hold last valid display
                }
                // Always reset state machine after WAIT_END, pass or fail
                _pstate = PSTATE_WAIT_START;
                break;

            default:
                _pstate = PSTATE_WAIT_START;
                break;
        }
    }
}

NanoFrame FRAMEPARSER_NANO_GetLatest(void) {
    return _latest_frame;
}

// ============================================================
// APP MAIN NANO
// ============================================================

void APP_NANO_Init(void) {
    HAL_UART_NANO_Init();

    // Wire.begin() called ONCE here — not inside any driver.
    // 400kHz fast mode: verify pull-up resistors <= 2.2kΩ for reliability.
    // If I2C errors occur at 400kHz, reduce to 100000UL.
    Wire.begin();
    Wire.setClock(400000UL);

    DRV_INA219_Init();
    DRV_OLED_Init();
    FRAMEPARSER_NANO_Init();
}

void APP_NANO_Run(void) {
    // Every loop — read UART bytes immediately, never skip
    FRAMEPARSER_NANO_Update();

    // Every 500ms — update power readings and refresh display
    uint32_t now = millis();
    if (now - _last_power_update >= INA219_UPDATE_INTERVAL_MS) {
        DRV_INA219_Update();
        PowerData power = DRV_INA219_GetData();
        DRV_OLED_DisplayPower(power);
        DRV_OLED_DisplayBatteryBar(power.voltage_v);
        // Redraw state row — persists after power row refresh
        DRV_OLED_DisplayState(_latest_frame.state_str);
        DRV_OLED_Update();
        _last_power_update = millis();
    }
}

// ============================================================
// TEST HOOKS — compiled ONLY when TEST_BUILD is defined
// ============================================================
#ifdef TEST_BUILD

// Expose _crc8 for unit testing
uint8_t TEST_crc8(const uint8_t* data, uint8_t len) {
    return _crc8(data, len);
}

// Allow test harness to override _last_power_update
// so timing-dependent branches can be exercised deterministically
void TEST_SetLastPowerUpdate(uint32_t t) {
    _last_power_update = t;
}

// SRAM free-space measurement (standard AVR trick)
int TEST_FreeRAM(void) {
    extern int __heap_start, *__brkval;
    int v;
    return (int)&v - (__brkval == 0
                      ? (int)&__heap_start
                      : (int)__brkval);
}

#endif // TEST_BUILD
