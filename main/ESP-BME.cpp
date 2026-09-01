/******************************************
 * ESP-BME - Weather station node
 *
 * ESP32-H2 (ESP32-H2FH4S) weather station exposing
 *   - Temperature  (standard Matter sensor)
 *   - Humidity     (standard Matter sensor)
 *   - Pressure     (standard Matter sensor)
 *   - Wind speed   (vendor custom cluster, uint16 in 0.1 m/s)
 *   - Rain amount  (vendor custom cluster, uint16 in 0.1 mm)
 *
 * Communication:  Matter over Thread
 * Commissioning:  BLE (manual pairing code / QR code)
 * Power saving:   Thread Sleepy End Device (MTD) + light sleep
 *
 * Built as an ESP-IDF project with the Arduino core as a component
 * (Arduino-as-IDF-component), see README.md.
 *******************************************/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <Matter.h>
#include <esp_matter.h>

// Thread Sleepy End Device
#include <platform/ConnectivityManager.h>
#include <esp_log.h>

// Power management (light sleep)
#include <esp_pm.h>

using namespace esp_matter;
using namespace chip::DeviceLayer;

// --------------------------------------------------------------------------
// Hardware configuration
// --------------------------------------------------------------------------
#define SEALEVELPRESSURE_HPA (1013.25)
#define I2C_SDA 4   // ESP32-H2: GPIO4 (SDA)
#define I2C_SCL 5   // ESP32-H2: GPIO5 (SCL)
#define ANEMOMETER ADC1_CH0  // GPIO36 on ESP32-H2 (ADC1_CH0)
#define INTERRUPT_PIN 12     // Rain gauge tipping-bucket counter input

// Sampling
#define SAMPLING_COUNT 10
#define SAMPLING_DELAY 100

// Rain gauge: one tip == 0.45 mm
#define RAIN_MM_PER_TIP 0.45

// Thread Sleepy End Device poll period in milliseconds
#define THREAD_POLL_INTERVAL_MS 5000

static const char *TAG = "esp-bme";

// --------------------------------------------------------------------------
// Vendor custom cluster for Wind + Rain
//   Cluster ID = 0x131BFC01  (MSB 16 bits: vendor 0x131B = Espressif test VID,
//                             LSB 16 bits: self-assigned)
//   Attribute 0x0000: wind_speed  uint16, value = m/s * 10  (0.1 m/s)
//   Attribute 0x0001: rain_amount uint16, value = mm  * 10  (0.1 mm)
// --------------------------------------------------------------------------
#define CUSTOM_CLUSTER_ID  0x131BFC01
#define ATTR_WIND_SPEED    0x0000
#define ATTR_RAIN_AMOUNT   0x0001
#define AMBIENT_ATTR_FLAGS ATTRIBUTE_FLAG_NONE

static attribute_t *windSpeedAttr = nullptr;
static attribute_t *rainAmountAttr = nullptr;
static uint16_t weatherEndpointId = 0;  // endpoint hosting the custom cluster
static volatile uint32_t rainTicks = 0;  // read from ISR, drained in loop()

// --------------------------------------------------------------------------
// Matter standard sensor endpoints
// --------------------------------------------------------------------------
MatterTemperatureSensor MatterTemp;
MatterHumiditySensor MatterHumidity;
MatterPressureSensor MatterPressure;

// --------------------------------------------------------------------------
// Sensors
// --------------------------------------------------------------------------
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;  // I2C only available on GPIO 4+5

/**
 * Rain gauge ISR - counts tipping-bucket tips (hardware debounced)
 */
IRAM_ATTR void rainSensorInterrupt() {
  rainTicks++;
}

/**
 * Create the vendor custom cluster (Wind + Rain) on a dedicated endpoint
 * of the (single) Matter node. Call once, after Matter.begin().
 */
static void createWeatherCustomCluster() {
  node_t *rootNode = node::get();

  endpoint_t *weatherEp = endpoint::create(rootNode, ENDPOINT_FLAG_NONE, nullptr);
  if (weatherEp == nullptr) {
    ESP_LOGE(TAG, "Failed to create weather endpoint");
    return;
  }
  weatherEndpointId = endpoint::get_id(weatherEp);

  cluster_t *cluster = cluster::create(weatherEp, CUSTOM_CLUSTER_ID, CLUSTER_FLAG_SERVER);
  if (cluster == nullptr) {
    ESP_LOGE(TAG, "Failed to create weather custom cluster");
    return;
  }

  windSpeedAttr = attribute::create(cluster, ATTR_WIND_SPEED, AMBIENT_ATTR_FLAGS, esp_matter_uint16(0));
  rainAmountAttr = attribute::create(cluster, ATTR_RAIN_AMOUNT, AMBIENT_ATTR_FLAGS, esp_matter_uint16(0));

  ESP_LOGI(TAG, "Weather custom cluster created (ep=%u)", weatherEndpointId);
}

/**
 * Publish a uint16 attribute of the weather custom cluster.
 */
static void setWeatherAttribute(uint32_t attributeId, uint16_t value) {
  if (weatherEndpointId == 0 || windSpeedAttr == nullptr || rainAmountAttr == nullptr) {
    return;
  }
  esp_matter_attr_val_t val = esp_matter_uint16(value);
  attribute::update(weatherEndpointId, CUSTOM_CLUSTER_ID, attributeId, &val);
}

/**
 * Sample BME280 + anemometer once.
 */
static void readSensors(float *temp, float *pressHpa, float *hum, float *windMs) {
  *temp = bme.readTemperature();
  *pressHpa = bme.readPressure() / 100.0F;
  *hum = bme.readHumidity();

  double wVoltage = analogRead(ANEMOMETER);
  // ESP32-H2: 12-bit ADC (0-4095), 3.3V reference, full scale maps to 25 m/s
  *windMs = (float)((wVoltage / 4095.0) * 3.3 * 25);
}

/**
 * Average a set of samples for stable readings.
 */
static void sampleAndAverage(float *t, float *p, float *h, float *w) {
  float st = 0, sp = 0, sh = 0, sw = 0;
  for (int i = 0; i < SAMPLING_COUNT; i++) {
    float temp, press, hum, wind;
    readSensors(&temp, &press, &hum, &wind);
    st += temp;
    sp += press;
    sh += hum;
    sw += wind;
    delay(SAMPLING_DELAY);
  }
  *t = st / SAMPLING_COUNT;
  *p = sp / SAMPLING_COUNT;
  *h = sh / SAMPLING_COUNT;
  *w = sw / SAMPLING_COUNT;
}

/**
 * Enable light sleep so the CPU sleeps between Thread polls (Sleepy End Device).
 */
static void enableLightSleep() {
  esp_pm_config_t pmConfig = {
      .max_freq_mhz = 96,   // ESP32-H2 top frequency
      .min_freq_mhz = 32,   // low power clock available for light sleep
      .light_sleep_enable = true,
  };
  esp_err_t err = esp_pm_configure(&pmConfig);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Light sleep not enabled (%s)", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "Light sleep enabled (Thread Sleepy End Device)");
  }
}

/**
 * Configure the Thread stack so this node behaves as a Sleepy End Device.
 * The radio is powered down between data polls, saving considerable energy.
 */
static void enableSleepyEndDevice() {
  ConnectivityMgr().SetThreadDeviceType(ConnectivityManager::kThreadDeviceType_SynchronizedSleepyEndDevice);
  // The default sleep interval already covers battery friendly polling; the
  // poll period can be tuned here if needed via Thread API.

  // Radio powered down between polls -> use light sleep to also sleep the CPU
  enableLightSleep();

  ESP_LOGI(TAG, "Configured as Thread Sleepy End Device");
}

void setup() {
  Serial.begin(115200);

  // I2C bus for BME280
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);

  // Rain gauge interrupt (hardware-debounced tipping bucket)
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), rainSensorInterrupt, FALLING);

  // BME280 sensor
  if (!bme.begin(0x76, &I2CBME)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1) delay(1000);
  }

  // Matter standard sensor endpoints
  MatterTemp.begin(0.0f);
  MatterHumidity.begin(0.0f);
  MatterPressure.begin(0.0f);

  // Start Matter (commissioning via BLE over Thread)
  Matter.begin();

  // Vendor custom cluster for Wind + Rain must be created after Matter.begin()
  createWeatherCustomCluster();

  // Wait for commissioning if the device has not been set up yet
  if (!Matter.isDeviceCommissioned()) {
    ESP_LOGI(TAG, "Matter Node is not commissioned yet.");
    ESP_LOGI(TAG, "Manual pairing code: %s", Matter.getManualPairingCode().c_str());
    ESP_LOGI(TAG, "QR code URL:        %s", Matter.getOnboardingQRCodeUrl().c_str());
    uint32_t waitIter = 0;
    while (!Matter.isDeviceCommissioned()) {
      delay(100);
      if ((waitIter++ % 50) == 0) {
        ESP_LOGI(TAG, "Waiting for commissioning...");
      }
    }
    ESP_LOGI(TAG, "Matter Node is commissioned and on the network.");
  }

  // Switch to Sleepy End Device mode once attached to the Thread network
  if (Matter.isThreadConnected()) {
    enableSleepyEndDevice();
  } else {
    ESP_LOGW(TAG, "Thread not connected yet, staying awake for now");
  }

  ESP_LOGI(TAG, "Setup complete.");
}

void loop() {
  float temp, press, hum, wind;
  sampleAndAverage(&temp, &press, &hum, &wind);

  // Standard Matter sensors
  MatterTemp.setTemperature(temp);
  MatterHumidity.setHumidity(hum);
  MatterPressure.setPressure(press);

  // Custom cluster: wind (m/s * 10) and accumulated rain (mm * 10)
  uint16_t windX10 = (uint16_t)(wind * 10.0f + 0.5f);
  setWeatherAttribute(ATTR_WIND_SPEED, windX10);

  // Drain rain ticks accumulated since last cycle
  uint32_t ticks = 0;
  {
    noInterrupts();
    ticks = rainTicks;
    rainTicks = 0;
    interrupts();
  }
  static uint32_t accumulatedRainMM10 = 0;
  accumulatedRainMM10 += (uint16_t)(ticks * RAIN_MM_PER_TIP * 10.0f);
  setWeatherAttribute(ATTR_RAIN_AMOUNT, accumulatedRainMM10);

  ESP_LOGI(TAG, "T %.1f C | H %.1f %% | P %.1f hPa | Wind %.1f m/s | Rain %.1f mm",
           temp, hum, press, wind, (float)accumulatedRainMM10 / 10.0f);

  delay(5000);
}
