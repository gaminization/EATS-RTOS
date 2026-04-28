/*
 * ============================================================
 * FILE: baseline_freertos_fixed.ino
 * TITLE: Baseline FreeRTOS IoT System — Fixed 240 MHz
 * BOARD: ESP32 Dev Module (Arduino IDE)
 *
 * FIXES APPLIED:
 *  - PIN_HEARTBEAT moved from GPIO 25 (ADC2, blocked by WiFi)
 *    to GPIO 36 (VP, ADC1, safe with WiFi active)
 *  - WiFi connection loop is bounded (max 20 retries, 10 s total)
 *    so taskMQTTPublish never hangs the scheduler indefinitely
 *  - Uninitialized local vars t/h/bpm/flame in MQTT task given
 *    safe default values before the mutex guard
 *  - lcd.printf() replaced with lcd.print(buf) — LiquidCrystal_I2C
 *    does not implement printf(); snprintf to a local buffer is used
 *  - logEnergy() called from inside the mutex guard removed to keep
 *    critical sections short; log is now called AFTER releasing mutex
 *  - Stack sizes raised to safe values:
 *      Sensor 3072, LCD 3072, MQTT 4096, Alarm 2048
 *  - TASK_PRIORITY raised to 2 so tasks are above the idle task
 *    and the loop() watchdog feed task
 *  - Wire.begin() moved before lcd.init() — LCD task now calls
 *    lcd.init() once at task start (Wire must already be up)
 *  - while(!Serial) removed — hangs forever on battery-powered boards
 * ============================================================
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ── Pins ────────────────────────────────────────────────────
#define PIN_FLAME      34   // ADC1_CH6
#define PIN_PIR        35   // ADC1_CH7
#define PIN_SOUND      32   // ADC1_CH4
#define PIN_LDR        33   // ADC1_CH5
#define PIN_DHT        26
#define PIN_HEARTBEAT  36   // VP / ADC1_CH0  ← safe with WiFi
#define PIN_RGB_R      18
#define PIN_RGB_G      19
#define PIN_RGB_B       5
#define PIN_BUZZER     27
#define DHT_TYPE       DHT22
#define LCD_ADDR       0x27

// ── Network ─────────────────────────────────────────────────
#define WIFI_SSID    "YOUR_SSID"
#define WIFI_PASS    "YOUR_PASSWORD"
#define MQTT_BROKER  "192.168.1.100"
#define MQTT_PORT    1883
#define MQTT_TOPIC   "iot/sensors"
#define MQTT_CLIENT  "esp32_baseline"

// ── Baseline config ─────────────────────────────────────────
#define FIXED_CPU_MHZ  240
#define TASK_PRIORITY  2        // above idle (0) and loop watchdog (1)

// ── Stack sizes (words on ESP32 = bytes / 4 internally,
//    but xTaskCreate takes bytes for Arduino-ESP32 core) ─────
#define STACK_SENSOR  3072
#define STACK_LCD     3072
#define STACK_MQTT    4096
#define STACK_ALARM   2048

// ── Simulated current draw ──────────────────────────────────
#define CURRENT_IDLE_MA    180.0f
#define CURRENT_WIFI_MA    320.0f
#define CURRENT_SENSOR_MA  200.0f

// ── Shared sensor data ──────────────────────────────────────
typedef struct {
    float    temperature;
    float    humidity;
    int      heartBPM;
    bool     flameDetected;
    bool     pirDetected;
    bool     soundDetected;
    int      ldrValue;
} SensorData_t;

static SensorData_t      g_sensors  = {};
static SemaphoreHandle_t g_dataMtx  = NULL;
static uint32_t          g_logStart = 0;

// ── Peripherals ─────────────────────────────────────────────
static DHT              dht(PIN_DHT, DHT_TYPE);
static LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
static WiFiClient       wifiClient;
static PubSubClient     mqttClient(wifiClient);

// ── Thread-safe serial log ───────────────────────────────────
static void energyLog(float mA, const char *proto) {
    uint32_t ms = millis() - g_logStart;
    Serial.printf("[LOG] %lu,%.1f,%s,%d\n", ms, mA, proto, FIXED_CPU_MHZ);
}

// ============================================================
// TASK: Sensor Read — Core 1, 500 ms period
// ============================================================
static void taskSensorRead(void *pv) {
    pinMode(PIN_FLAME, INPUT);
    pinMode(PIN_PIR,   INPUT);
    pinMode(PIN_SOUND, INPUT);
    dht.begin();

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        float t = dht.readTemperature();
        float h = dht.readHumidity();

        // ADC1 reads only — safe with WiFi driver active
        int  rawFlame = analogRead(PIN_FLAME);
        int  rawSound = analogRead(PIN_SOUND);
        int  rawLDR   = analogRead(PIN_LDR);
        int  rawHB    = analogRead(PIN_HEARTBEAT);
        bool pir      = (digitalRead(PIN_PIR) == HIGH);

        if (xSemaphoreTake(g_dataMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
            g_sensors.temperature  = isnan(t) ? 0.0f : t;
            g_sensors.humidity     = isnan(h) ? 0.0f : h;
            g_sensors.heartBPM     = (int)map(rawHB, 0, 4095, 40, 120);
            g_sensors.flameDetected = (rawFlame < 1500);
            g_sensors.soundDetected = (rawSound > 2000);
            g_sensors.pirDetected   = pir;
            g_sensors.ldrValue      = rawLDR;
            xSemaphoreGive(g_dataMtx);
        }

        energyLog(CURRENT_SENSOR_MA, "SENSOR");
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(500));
    }
}

// ============================================================
// TASK: LCD Update — Core 1, 1000 ms period
// ============================================================
static void taskLCDUpdate(void *pv) {
    lcd.init();
    lcd.backlight();

    char row0[17];
    char row1[17];

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        float t   = 0.0f;
        float h   = 0.0f;
        int   bpm = 0;
        bool  flame = false;

        if (xSemaphoreTake(g_dataMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
            t     = g_sensors.temperature;
            h     = g_sensors.humidity;
            bpm   = g_sensors.heartBPM;
            flame = g_sensors.flameDetected;
            xSemaphoreGive(g_dataMtx);
        }

        if (flame) {
            lcd.setCursor(0, 0); lcd.print("!! FIRE ALERT !!");
            lcd.setCursor(0, 1); lcd.print("CHECK SENSOR!   ");
        } else {
            snprintf(row0, sizeof(row0), "T:%.1fC H:%.0f%%   ", t, h);
            snprintf(row1, sizeof(row1), "BPM:%3d  240MHz ", bpm);
            lcd.setCursor(0, 0); lcd.print(row0);
            lcd.setCursor(0, 1); lcd.print(row1);
        }

        energyLog(CURRENT_IDLE_MA, "LCD");
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));
    }
}

// ============================================================
// TASK: MQTT Publish — Core 0, 5000 ms period
// ============================================================
static void taskMQTTPublish(void *pv) {
    // Non-blocking WiFi connect: max 10 s then continue
    Serial.println("[MQTT] Connecting WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[MQTT] WiFi connected: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[MQTT] WiFi connect failed — will retry each cycle");
    }

    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setKeepAlive(15);

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        // Reconnect WiFi if lost
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.reconnect();
            vTaskDelay(pdMS_TO_TICKS(2000));
        }

        // Reconnect MQTT (single attempt per cycle — non-blocking)
        if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
            Serial.println("[MQTT] Connecting broker...");
            mqttClient.connect(MQTT_CLIENT);
        }
        mqttClient.loop();

        // Read sensor snapshot
        float t   = 0.0f;
        float h   = 0.0f;
        int   bpm = 0;
        bool  flame = false;
        bool  pir   = false;
        int   ldr   = 0;

        if (xSemaphoreTake(g_dataMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
            t     = g_sensors.temperature;
            h     = g_sensors.humidity;
            bpm   = g_sensors.heartBPM;
            flame = g_sensors.flameDetected;
            pir   = g_sensors.pirDetected;
            ldr   = g_sensors.ldrValue;
            xSemaphoreGive(g_dataMtx);
        }

        if (mqttClient.connected()) {
            char payload[128];
            snprintf(payload, sizeof(payload),
                     "{\"T\":%.1f,\"H\":%.1f,\"BPM\":%d,\"flame\":%d,\"pir\":%d,\"ldr\":%d}",
                     t, h, bpm, (int)flame, (int)pir, ldr);
            mqttClient.publish(MQTT_TOPIC, payload);
            Serial.printf("[MQTT] Published → %s\n", payload);
            energyLog(CURRENT_WIFI_MA, "WIFI");
        } else {
            energyLog(CURRENT_IDLE_MA, "IDLE");
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(5000));
    }
}

// ============================================================
// TASK: Alarm Handler — Core 1, 200 ms period
// ============================================================
static void taskAlarmHandler(void *pv) {
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_RGB_R,  OUTPUT);
    pinMode(PIN_RGB_G,  OUTPUT);
    pinMode(PIN_RGB_B,  OUTPUT);

    // Safe initial state
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_RGB_R,  LOW);
    digitalWrite(PIN_RGB_G,  HIGH);
    digitalWrite(PIN_RGB_B,  LOW);

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        bool flame = false;
        bool sound = false;

        if (xSemaphoreTake(g_dataMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
            flame = g_sensors.flameDetected;
            sound = g_sensors.soundDetected;
            xSemaphoreGive(g_dataMtx);
        }

        if (flame) {
            digitalWrite(PIN_BUZZER, HIGH);
            digitalWrite(PIN_RGB_R,  HIGH);
            digitalWrite(PIN_RGB_G,  LOW);
            digitalWrite(PIN_RGB_B,  LOW);
            Serial.println("[ALARM] *** FIRE DETECTED ***");
        } else if (sound) {
            digitalWrite(PIN_BUZZER, LOW);
            digitalWrite(PIN_RGB_R,  LOW);
            digitalWrite(PIN_RGB_G,  LOW);
            digitalWrite(PIN_RGB_B,  HIGH);
            // tone() is not FreeRTOS-safe for long durations;
            // use a brief fixed-frequency toggle instead
            digitalWrite(PIN_BUZZER, HIGH);
            vTaskDelay(pdMS_TO_TICKS(50));
            digitalWrite(PIN_BUZZER, LOW);
        } else {
            digitalWrite(PIN_BUZZER, LOW);
            digitalWrite(PIN_RGB_R,  LOW);
            digitalWrite(PIN_RGB_G,  HIGH);
            digitalWrite(PIN_RGB_B,  LOW);
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(200));
    }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    // Do NOT call while(!Serial) — hangs on battery boards

    Wire.begin(21, 22);  // must be before any I2C device

    setCpuFrequencyMhz(FIXED_CPU_MHZ);

    g_dataMtx = xSemaphoreCreateMutex();
    configASSERT(g_dataMtx != NULL);

    g_logStart = millis();

    Serial.println();
    Serial.println("==============================================");
    Serial.println(" BASELINE  |  Fixed 240 MHz  |  Equal Prio  ");
    Serial.println("==============================================");
    Serial.println("[LOG] TIME_MS,CURRENT_MA,PROTOCOL,CPU_FREQ_MHZ");

    xTaskCreatePinnedToCore(taskSensorRead,   "Sensors", STACK_SENSOR, NULL, TASK_PRIORITY, NULL, 1);
    xTaskCreatePinnedToCore(taskLCDUpdate,    "LCD",     STACK_LCD,    NULL, TASK_PRIORITY, NULL, 1);
    xTaskCreatePinnedToCore(taskAlarmHandler, "Alarm",   STACK_ALARM,  NULL, TASK_PRIORITY, NULL, 1);
    xTaskCreatePinnedToCore(taskMQTTPublish,  "MQTT",    STACK_MQTT,   NULL, TASK_PRIORITY, NULL, 0);

    Serial.println("[BASELINE] All tasks created.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(5000));
}
