// ============================================================
// ESP32_INTEGRATED.ino — Single Entry Point for ESP32 WROOM-32
// Project  : Waiter Robot — ESP32_MAIN
// Board    : ESP32 WROOM-32 38-pin
//
// BUILD MODES — uncomment ONE define to select build mode:
//   (default, both commented) = PRODUCTION: runs APP_MAIN_Run() robot firmware
//   HAL_TEST_BUILD             = HAL layer test suite (RunAllTests, 21 tests)
//   DRV_TEST_BUILD             = Driver layer test suite (8 hardware tests)
//
// HOW TO USE:
//   1. Put ESP32_INTEGRATED.h, ESP32_INTEGRATED.cpp, ESP32_INTEGRATED.ino
//      in ONE folder named ESP32_INTEGRATED.
//   2. Open ESP32_INTEGRATED.ino in Arduino IDE.
//   3. Board: ESP32 Dev Module, CPU: 240MHz, Flash: 4MB.
//   4. Leave both defines commented for production robot firmware.
//   5. Uncomment HAL_TEST_BUILD or DRV_TEST_BUILD for bench testing.
// ============================================================

// Uncomment ONE for test builds. Leave BOTH commented for production.
// #define HAL_TEST_BUILD
// #define DRV_TEST_BUILD

#include "ESP32_INTEGRATED.h"

// ============================================================
// STRINGIFY helper — required by HAL_TEST_BUILD setup()
// Must be defined before setup() is compiled.
// ============================================================
#define STRINGIFY_INNER(x) #x
#define STRINGIFY(x)       STRINGIFY_INNER(x)

// ============================================================
// ── HAL TEST BUILD ───────────────────────────────────────────
// Runs 21 HAL-layer tests (SYSTEM, I2C, UART, ADC, LEDC).
// Output goes to Serial (USB) at 115200 baud.
// ============================================================
#ifdef HAL_TEST_BUILD

void setup(void) {
    HAL_UART_Init(UART_PORT_DEBUG);
    delay(500);

    Serial.println(F("\n[BOOT] HAL_MT1 — Waiter Robot ESP32_MAIN"));
    Serial.printf("[BOOT] ESP-IDF %s | Arduino Core\n", esp_get_idf_version());

    Serial.println(F("[BOOT] HAL_SYSTEM_Init ..."));
    HAL_SYSTEM_Init();
    Serial.printf("[BOOT] CPU @ %dMHz | WDT @ %dms\n",
                  CPU_FREQ_MHZ, WATCHDOG_TIMEOUT_MS);

    Serial.println(F("[BOOT] HAL_I2C_Init ..."));
    HAL_I2C_Init();

    Serial.println(F("[BOOT] HAL_UART_Init CAM + Nano ..."));
    HAL_UART_Init(UART_PORT_CAM);
    HAL_UART_Init(UART_PORT_NANO);

    Serial.println(F("[BOOT] HAL_ADC_Init ..."));
    HAL_ADC_Init();

    Serial.println(F("[BOOT] HAL_LEDC_Init ..."));
    HAL_LEDC_Init();

    Serial.println(F("[BOOT] All HAL modules initialised.\n"));

    RunAllTests();

    Serial.println(F("[BOOT] Entering main loop (MAIN_LOOP_PERIOD_MS="
                     STRINGIFY(MAIN_LOOP_PERIOD_MS) "ms)\n"));
}

void loop(void) {
    uint32_t loop_start = HAL_SYSTEM_GetTick();

    HAL_SYSTEM_FeedWatchdog();

    while (HAL_UART_BytesAvailable(UART_PORT_CAM)) {
        (void)HAL_UART_ReadByte(UART_PORT_CAM);
    }

    static uint32_t last_telem = 0;
    if ((loop_start - last_telem) >= 2000) {
        last_telem = loop_start;
        float    vbat = HAL_ADC_ReadBatteryVoltage();
        uint32_t tick = HAL_SYSTEM_GetTick();
        Serial.printf("[LOOP] tick=%ums | vbat=%.3fV | sys=%d | adc=%d | ledc=%d\n",
                      tick, vbat,
                      (int)HAL_SYSTEM_GetStatus(),
                      (int)HAL_ADC_GetStatus(),
                      (int)HAL_LEDC_GetStatus());
    }

    uint32_t elapsed = HAL_SYSTEM_GetTick() - loop_start;
    if (elapsed < MAIN_LOOP_PERIOD_MS) {
        HAL_SYSTEM_DelayMs(MAIN_LOOP_PERIOD_MS - elapsed);
    }
}

// ============================================================
// ── DRIVER TEST BUILD ────────────────────────────────────────
// Runs 8 driver hardware tests (MPU9250, AK8963, TCS3200,
// HC-SR04, Line Array, Encoders, L298N Motors, Battery).
// Requires all sensors wired. Output to Serial at 115200 baud.
// Send 'y'/'n' to confirm manual/visual test results.
// ============================================================
#elif defined(DRV_TEST_BUILD)

// ── Test result helpers ──────────────────────────────────────
static void print_result(const char* label, bool pass) {
    Serial.printf("  %-40s %s\n", label, pass ? "PASS" : "FAIL");
}

static bool confirm_manual(const char* label, uint32_t timeout_ms = 8000) {
    Serial.printf("  [CONFIRM] %s  (y=PASS / n=FAIL, %us timeout)\n",
                  label, timeout_ms / 1000);
    uint32_t deadline = HAL_SYSTEM_GetTick() + timeout_ms;
    while (HAL_SYSTEM_GetTick() < deadline) {
        HAL_SYSTEM_FeedWatchdog();
        if (Serial.available()) {
            char c = (char)Serial.read();
            if (c == 'y' || c == 'Y') return true;
            if (c == 'n' || c == 'N') return false;
        }
        HAL_SYSTEM_DelayMs(50);
    }
    Serial.println(F("  [TIMEOUT] No input — marking FAIL"));
    return false;
}

// ── TEST 1 — MPU9250 ─────────────────────────────────────────
static void runMPU9250Test(void) {
    Serial.println(F("\n--- TEST 1: MPU9250 ---"));
    uint8_t who = 0;
    HAL_I2C_ReadByte(MPU9250_I2C_ADDR, MPU9250_WHO_AM_I_REG, &who);
    print_result("WHO_AM_I == 0x71", who == MPU9250_WHO_AM_I_VAL);
    MPU_Status st = DRV_MPU9250_Init();
    print_result("Init returns MPU_OK", st == MPU_OK);
    MPU_Status ust = DRV_MPU9250_Update();
    print_result("Update returns MPU_OK", ust == MPU_OK);
    IMUData d = DRV_MPU9250_GetData();
    bool heading_valid = (d.heading_deg >= 0.0f && d.heading_deg <= 360.0f);
    print_result("Heading in range 0-360", heading_valid);
    Serial.println(F("  [MANUAL] Rotate robot — watch heading change below:"));
    for (uint8_t i = 0; i < 10; i++) {
        DRV_MPU9250_Update();
        Serial.printf("  heading=%.2f  pitch=%.2f  roll=%.2f\n",
                      DRV_MPU9250_GetHeading(),
                      DRV_MPU9250_GetPitch(),
                      DRV_MPU9250_GetRoll());
        HAL_SYSTEM_DelayMs(200);
        HAL_SYSTEM_FeedWatchdog();
    }
    HAL_SYSTEM_FeedWatchdog();
}

// ── TEST 2 — AK8963 ──────────────────────────────────────────
static void runAK8963Test(void) {
    Serial.println(F("\n--- TEST 2: AK8963 ---"));
    uint8_t who = 0;
    HAL_I2C_ReadByte(AK8963_I2C_ADDR, AK8963_WHO_AM_I_REG, &who);
    print_result("AK8963 WHO_AM_I == 0x48", who == AK8963_WHO_AM_I_VAL);
    AK_Status st = DRV_AK8963_Init();
    print_result("AK8963 Init returns AK_OK", st == AK_OK);
    AK_Status ust = AK_NOT_READY;
    uint32_t t0 = HAL_SYSTEM_GetTick();
    while ((HAL_SYSTEM_GetTick() - t0) < 20) {
        ust = DRV_AK8963_Update();
        if (ust == AK_OK) break;
        HAL_SYSTEM_DelayMs(1);
    }
    print_result("AK8963 Update gets data within 20ms", ust == AK_OK);
    float h = DRV_AK8963_GetHeading();
    print_result("Mag heading in range 0-360", h >= 0.0f && h <= 360.0f);
    Serial.println(F("  [MANUAL] Rotate robot to see heading sweep 0-360:"));
    for (uint8_t i = 0; i < 10; i++) {
        DRV_AK8963_Update();
        Serial.printf("  mag_heading=%.2f deg\n", DRV_AK8963_GetHeading());
        HAL_SYSTEM_DelayMs(200);
        HAL_SYSTEM_FeedWatchdog();
    }
    HAL_SYSTEM_FeedWatchdog();
}

// ── TEST 3 — TCS3200 ─────────────────────────────────────────
static void runTCS3200Test(void) {
    Serial.println(F("\n--- TEST 3: TCS3200 ---"));
    DRV_TCS3200_Init();
    print_result("TCS3200 Init (no crash)", true);
    DRV_TCS3200_Update();
    HAL_SYSTEM_FeedWatchdog();
    uint32_t r = DRV_TCS3200_GetChannel(TCS_CH_RED);
    uint32_t g = DRV_TCS3200_GetChannel(TCS_CH_GREEN);
    uint32_t b = DRV_TCS3200_GetChannel(TCS_CH_BLUE);
    Serial.printf("  Raw counts -> R=%u  G=%u  B=%u\n", r, g, b);
    Serial.println(F("  [MANUAL] Hold RED card under sensor..."));
    HAL_SYSTEM_DelayMs(3000); HAL_SYSTEM_FeedWatchdog();
    DRV_TCS3200_Update(); HAL_SYSTEM_FeedWatchdog();
    print_result("RED card detected as TCS_RED", DRV_TCS3200_GetColor() == TCS_RED);
    Serial.println(F("  [MANUAL] Hold BLACK card (or black tape) under sensor..."));
    HAL_SYSTEM_DelayMs(3000); HAL_SYSTEM_FeedWatchdog();
    DRV_TCS3200_Update(); HAL_SYSTEM_FeedWatchdog();
    print_result("BLACK detected as TCS_BLACK", DRV_TCS3200_GetColor() == TCS_BLACK);
    Serial.println(F("  [MANUAL] Hold WHITE card under sensor..."));
    HAL_SYSTEM_DelayMs(3000); HAL_SYSTEM_FeedWatchdog();
    DRV_TCS3200_Update(); HAL_SYSTEM_FeedWatchdog();
    print_result("WHITE detected as TCS_WHITE", DRV_TCS3200_GetColor() == TCS_WHITE);
    bool is_line = DRV_TCS3200_IsLineColor();
    Serial.printf("  IsLineColor()=%s (target=%s)\n",
                  is_line ? "true" : "false",
                  (LINE_COLOR_TARGET == TCS_BLACK) ? "BLACK" : "WHITE");
    HAL_SYSTEM_FeedWatchdog();
}

// ── TEST 4 — HC-SR04 ─────────────────────────────────────────
static void runHCSR04Test(void) {
    Serial.println(F("\n--- TEST 4: HC-SR04 ---"));
    DRV_HCSR04_Init();
    print_result("HCSR04 Init (no crash)", true);
    Serial.println(F("  [MANUAL] Place object at ~20cm in front of sensor..."));
    HAL_SYSTEM_DelayMs(2000); HAL_SYSTEM_FeedWatchdog();
    float d = DRV_HCSR04_GetDistance();
    bool pass_20 = (d >= 18.0f && d <= 22.0f);
    Serial.printf("  Distance=%.2fcm (expect 18-22cm): %s\n", d, pass_20 ? "PASS" : "FAIL");
    print_result("Object at 20cm +-2cm", pass_20);
    Serial.println(F("  [MANUAL] Remove object (clear path)..."));
    HAL_SYSTEM_DelayMs(2000); HAL_SYSTEM_FeedWatchdog();
    float d2 = DRV_HCSR04_GetDistance();
    bool pass_oor = (DRV_HCSR04_GetStatus() == HCSR_TIMEOUT ||
                     DRV_HCSR04_GetStatus() == HCSR_OUT_OF_RANGE || d2 > 200.0f);
    Serial.printf("  Clear distance=%.2fcm: %s\n", d2, pass_oor ? "PASS" : "FAIL");
    print_result("No object -> out-of-range/timeout", pass_oor);
    Serial.println(F("  [MANUAL] Place object < 15cm in front..."));
    HAL_SYSTEM_DelayMs(2000); HAL_SYSTEM_FeedWatchdog();
    DRV_HCSR04_GetDistance();
    bool obs = DRV_HCSR04_ObstacleDetected();
    print_result("ObstacleDetected() = true at <15cm", obs);
    HAL_SYSTEM_FeedWatchdog();
}

// ── TEST 5 — LINE ARRAY ───────────────────────────────────────
static void runLineArrayTest(void) {
    Serial.println(F("\n--- TEST 5: LINE ARRAY ---"));
    DRV_LINE_Init();
    print_result("LINE Init (no crash)", true);
    Serial.println(F("  [MANUAL] Place robot on line, centered (S3 on line)..."));
    HAL_SYSTEM_DelayMs(2000); HAL_SYSTEM_FeedWatchdog();
    DRV_LINE_Update();
    int8_t err = DRV_LINE_GetError();
    Serial.printf("  Center -> error=%d (expect 0): %s\n", err, (err == 0) ? "PASS" : "FAIL");
    print_result("Center line -> error == 0", err == 0);
    Serial.println(F("  [MANUAL] Shift robot far LEFT (S1 on line)..."));
    HAL_SYSTEM_DelayMs(2000); HAL_SYSTEM_FeedWatchdog();
    DRV_LINE_Update(); err = DRV_LINE_GetError();
    Serial.printf("  Full left -> error=%d (expect -2): %s\n", err, (err == LINE_ERROR_FULL_LEFT) ? "PASS" : "FAIL");
    print_result("Full left -> error == -2", err == LINE_ERROR_FULL_LEFT);
    Serial.println(F("  [MANUAL] Shift robot far RIGHT (S5 on line)..."));
    HAL_SYSTEM_DelayMs(2000); HAL_SYSTEM_FeedWatchdog();
    DRV_LINE_Update(); err = DRV_LINE_GetError();
    Serial.printf("  Full right -> error=%d (expect +2): %s\n", err, (err == LINE_ERROR_FULL_RIGHT) ? "PASS" : "FAIL");
    print_result("Full right -> error == +2", err == LINE_ERROR_FULL_RIGHT);
    Serial.println(F("  [MANUAL] Lift robot off line (all sensors off line)..."));
    HAL_SYSTEM_DelayMs(2000); HAL_SYSTEM_FeedWatchdog();
    DRV_LINE_Update();
    bool lost = DRV_LINE_IsLost();
    Serial.printf("  Line lost=%s (expect true): %s\n", lost ? "true" : "false", lost ? "PASS" : "FAIL");
    print_result("All sensors off -> line_lost = true", lost);
    HAL_SYSTEM_FeedWatchdog();
}

// ── TEST 6 — ENCODER ─────────────────────────────────────────
static void runEncoderTest(void) {
    Serial.println(F("\n--- TEST 6: ENCODERS ---"));
    DRV_ENCODER_Init(); DRV_ENCODER_Reset();
    print_result("Encoder Init (no crash)", true);
    Serial.println(F("  [MANUAL] Manually trigger LEFT encoder pin 100 times..."));
    Serial.println(F("  (or spin left wheel until ~100 ticks)"));
    HAL_SYSTEM_DelayMs(5000); HAL_SYSTEM_FeedWatchdog();
    DRV_ENCODER_Update();
    EncoderData ed = DRV_ENCODER_GetData();
    bool tick_pass = (ed.left_ticks >= 99 && ed.left_ticks <= 101);
    Serial.printf("  Left ticks=%d (expect ~100): %s\n", ed.left_ticks, tick_pass ? "PASS" : "FAIL");
    print_result("100 ticks counted +-1", tick_pass);
    DRV_ENCODER_Reset();
    Serial.println(F("  [MANUAL] Spin left wheel briskly for 2 seconds..."));
    HAL_SYSTEM_DelayMs(2000); HAL_SYSTEM_FeedWatchdog();
    DRV_ENCODER_Update(); ed = DRV_ENCODER_GetData();
    bool rpm_pass = (ed.left_rpm > 0.0f);
    Serial.printf("  Left RPM=%.2f (expect >0): %s\n", ed.left_rpm, rpm_pass ? "PASS" : "FAIL");
    print_result("RPM nonzero during spin", rpm_pass);
    float dist_mm = DRV_ENCODER_GetDistanceMM();
    Serial.printf("  Distance since reset=%.2fmm\n", dist_mm);
    print_result("Distance > 0 after movement", dist_mm > 0.0f);
    DRV_ENCODER_Reset(); DRV_ENCODER_Update(); ed = DRV_ENCODER_GetData();
    print_result("Reset() clears ticks to 0", ed.left_ticks == 0 && ed.right_ticks == 0);
    HAL_SYSTEM_FeedWatchdog();
}

// ── TEST 7 — L298N MOTORS ────────────────────────────────────
static void runL298NTest(void) {
    Serial.println(F("\n--- TEST 7: L298N MOTORS ---"));
    DRV_L298N_Init();
    print_result("L298N Init (no crash)", true);
    Serial.println(F("  [MANUAL] Both motors should spin FORWARD for 1 second..."));
    DRV_L298N_SetMotors(MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    HAL_SYSTEM_DelayMs(1000); HAL_SYSTEM_FeedWatchdog();
    DRV_L298N_Stop();
    print_result("SetMotors(255,255) forward 1s", confirm_manual("Did BOTH motors spin FORWARD?"));
    HAL_SYSTEM_DelayMs(500); HAL_SYSTEM_FeedWatchdog();
    Serial.println(F("  [MANUAL] Both motors BACKWARD for 1 second..."));
    DRV_L298N_SetMotors(-MOTOR_MAX_SPEED, -MOTOR_MAX_SPEED);
    HAL_SYSTEM_DelayMs(1000); HAL_SYSTEM_FeedWatchdog();
    DRV_L298N_Stop();
    print_result("SetMotors(-255,-255) backward 1s", confirm_manual("Did BOTH motors spin BACKWARD?"));
    HAL_SYSTEM_DelayMs(500); HAL_SYSTEM_FeedWatchdog();
    DRV_L298N_SetMotors(0, 0); HAL_SYSTEM_DelayMs(200);
    print_result("SetMotors(0,0) -> stopped", confirm_manual("Are both motors STOPPED?", 5000));
    DRV_L298N_SetMotors(10, 10); HAL_SYSTEM_DelayMs(500); HAL_SYSTEM_FeedWatchdog();
    DRV_L298N_Stop();
    Serial.printf("  [INFO] Duty=10 < DEADBAND(%d) -> clamped to %d (verify scope)\n",
                  MOTOR_DEADBAND, MOTOR_DEADBAND);
    print_result("Deadband clamp applied (no stall)", confirm_manual("Motors moved without stalling?"));
    DRV_L298N_SetMotors(200, 200); HAL_SYSTEM_DelayMs(300); HAL_SYSTEM_FeedWatchdog();
    DRV_L298N_EmergencyBrake();
    print_result("EmergencyBrake() stops both motors", confirm_manual("Did motors STOP immediately?"));
    HAL_SYSTEM_FeedWatchdog();
}

// ── TEST 8 — BATTERY ─────────────────────────────────────────
static void runBatteryTest(void) {
    Serial.println(F("\n--- TEST 8: BATTERY MONITOR ---"));
    DRV_BATTERY_Init();
    print_result("Battery Init (no crash)", true);
    Serial.println(F("  [WAIT] Waiting 1.1s for battery interval gate..."));
    HAL_SYSTEM_DelayMs(1100); HAL_SYSTEM_FeedWatchdog();
    DRV_BATTERY_Update();
    BatteryData bd = DRV_BATTERY_GetData();
    bool v_pass = (bd.voltage > 2.0f && bd.voltage < 12.0f);
    Serial.printf("  Voltage=%.3fV: %s\n", bd.voltage, v_pass ? "PASS" : "FAIL");
    print_result("Battery voltage 2-12V range", v_pass);
    bool pct_pass = (bd.percent <= 100);
    Serial.printf("  Percent=%u%%: %s\n", bd.percent, pct_pass ? "PASS" : "FAIL");
    print_result("Battery percent 0-100", pct_pass);
    bool low_consistent  = (bd.is_low      == (bd.voltage < BATTERY_LOW_THRESHOLD_V));
    bool vlow_consistent = (bd.is_very_low == (bd.voltage < BATTERY_VERYLOW_THRESHOLD_V));
    print_result("IsLow flag consistent with threshold",     low_consistent);
    print_result("IsVeryLow flag consistent with threshold", vlow_consistent);
    float prev_v = bd.voltage;
    float max_step = BATTERY_FILTER_ALPHA * (BATTERY_FULL_V - BATTERY_VERYLOW_THRESHOLD_V) * 1.5f;
    bool filter_ok = true;
    for (uint8_t i = 0; i < 5; i++) {
        HAL_SYSTEM_DelayMs(1100); HAL_SYSTEM_FeedWatchdog();
        DRV_BATTERY_Update();
        float new_v = DRV_BATTERY_GetData().voltage;
        if (fabsf(new_v - prev_v) > max_step) filter_ok = false;
        prev_v = new_v;
    }
    print_result("IIR filter: voltage step <= alpha*range per cycle", filter_ok);
    Serial.printf("  [MANUAL] Verify voltage reading against multimeter (+-200mV)\n");
    Serial.printf("  Reported=%.3fV  Expected: multimeter reading\n", bd.voltage);
    HAL_SYSTEM_FeedWatchdog();
}

void setup(void) {
    Serial.begin(UART_DEBUG_BAUD);
    delay(500);

    Serial.println(F("\n============================================================"));
    Serial.println(F("  MT-2 DRIVER TEST SUITE — Waiter Robot ESP32_MAIN"));
    Serial.println(F("============================================================"));

    HAL_SYSTEM_Init();
    HAL_I2C_Init();
    HAL_UART_Init(UART_PORT_CAM);
    HAL_UART_Init(UART_PORT_NANO);
    HAL_ADC_Init();
    HAL_LEDC_Init();

    Serial.println(F("[BOOT] HAL layer ready. Starting driver tests...\n"));

    runMPU9250Test();  HAL_SYSTEM_FeedWatchdog();
    runAK8963Test();   HAL_SYSTEM_FeedWatchdog();
    runTCS3200Test();  HAL_SYSTEM_FeedWatchdog();
    runHCSR04Test();   HAL_SYSTEM_FeedWatchdog();
    runLineArrayTest(); HAL_SYSTEM_FeedWatchdog();
    runEncoderTest();  HAL_SYSTEM_FeedWatchdog();
    runL298NTest();    HAL_SYSTEM_FeedWatchdog();
    runBatteryTest();  HAL_SYSTEM_FeedWatchdog();

    Serial.println(F("\n============================================================"));
    Serial.println(F("  ALL TESTS COMPLETE — entering live telemetry loop"));
    Serial.println(F("  (500ms interval — use for calibration)"));
    Serial.println(F("============================================================\n"));
}

void loop(void) {
    uint32_t t0 = HAL_SYSTEM_GetTick();
    HAL_SYSTEM_FeedWatchdog();
    DRV_MPU9250_Update();
    DRV_AK8963_Update();
    DRV_ENCODER_Update();
    DRV_BATTERY_Update();
    static uint32_t last_print = 0;
    if ((t0 - last_print) >= 500) {
        last_print = t0;
        IMUData     imu = DRV_MPU9250_GetData();
        BatteryData bat = DRV_BATTERY_GetData();
        EncoderData enc = DRV_ENCODER_GetData();
        DRV_LINE_Update();
        LineData    lin = DRV_LINE_GetData();
        Serial.printf("[LIVE] t=%us | IMU hd=%.1f pt=%.1f rl=%.1f | MAG=%.1f | "
                      "BAT=%.2fV %u%% | ENC L=%d R=%d | LINE mask=0x%02X err=%d lost=%d\n",
                      t0 / 1000,
                      imu.heading_deg, imu.pitch_deg, imu.roll_deg,
                      imu.mag_heading_deg,
                      bat.voltage, bat.percent,
                      enc.left_ticks, enc.right_ticks,
                      lin.bitmask, lin.error, (int)lin.line_lost);
        float dist = DRV_HCSR04_GetDistance();
        HAL_SYSTEM_FeedWatchdog();
        Serial.printf("[LIVE] HCSR04=%.1fcm | OBS=%s\n",
                      dist, DRV_HCSR04_ObstacleDetected() ? "YES" : "NO");
    }
    if (DRV_BATTERY_IsVeryLow()) {
        DRV_L298N_EmergencyBrake();
        Serial.println(F("[CRITICAL] Battery very low! Emergency brake active."));
    }
    uint32_t elapsed = HAL_SYSTEM_GetTick() - t0;
    if (elapsed < 20) {
        HAL_SYSTEM_DelayMs(20 - elapsed);
    }
}

// ============================================================
// ── PRODUCTION BUILD (DEFAULT) ───────────────────────────────
// Full robot firmware. Runs APP_MAIN_Init() + APP_MAIN_Run().
// This is what gets flashed to the robot in the field.
// ============================================================
#else

void setup() {
    Serial.begin(115200);
    APP_MAIN_Init();
    Serial.println("ESP32 WAITER ROBOT — BOOT COMPLETE");
}

void loop() {
    APP_MAIN_Run();
}

#endif // build mode selector
