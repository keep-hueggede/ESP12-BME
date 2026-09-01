/******************************************
 * ESP-BME - Wetterstation über Thread + CoAP
 *
 * ESP32-H2 (ESP32-H2FH4S) Wetterknoten, der Messwerte über ein
 * Thread-Netzwerk bereitstellt. Der Thread Border Router (OTBR) im
 * Netz empfängt die Daten und leitet sie weiter (z. B. in eine DB).
 *
 * Übertragung:    Thread (801.15.4) + CoAP (UDP, Port 5683)
 * Rolle:          CoAP-Server, stellt die Messwerte als Ressourcen bereit
 *                 (das Backend liest sie per CoAP-GET über den OTBR)
 * Kommissionierung: Node tritt dem vom OTBR geformten Thread-Netz mit
 *                 einem Operational Dataset bei (siehe THREAD_* unten).
 *
 * Benötigt den Arduino-Core (arduino-esp32 v3.x) mit OpenThread-Unterstützung
 * (ESP32-H2/C6/C5). OpenThread muss im Core-Build aktiviert sein.
 *******************************************/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OThread.h>
#include <OThreadCoAP.h>

// --------------------------------------------------------------------------
// Hardware-Konfiguration
// --------------------------------------------------------------------------
#define SEALEVELPRESSURE_HPA (1013.25)
#define I2C_SDA 4          // ESP32-H2: GPIO4 (SDA)
#define I2C_SCL 5          // ESP32-H2: GPIO5 (SCL)
#define ANEMOMETER ADC1_CH0 // GPIO36 (ADC1_CH0) - Anemometer analog
#define INTERRUPT_PIN 12    // Regenmesser (Tipping Bucket), Interrupt

// Abtastung
#define SAMPLING_COUNT 10
#define SAMPLING_DELAY 100
#define REPORT_INTERVAL_MS 5000   // Intervall, in dem die Messwerte aktualisiert werden

// Regenmesser: 1 Kippung = 0.45 mm
#define RAIN_MM_PER_TIP 0.45

// --------------------------------------------------------------------------
// Thread-Netzkonfiguration (Operational Dataset)
//
// WICHTIG: Trage hier die Werte deines Thread-Netzwerks ein. Diese bekommst du
// vom Thread Border Router (z. B. Home Assistant -> Thread-Integration:
// Network Name, PAN ID, Channel, Extended PAN ID, Network Key).
// --------------------------------------------------------------------------
static const char    THREAD_NETWORK_NAME[] = "OpenThread-XXXX";      // z. B. "OpenThread-demo"
static const uint16_t THREAD_PAN_ID        = 0x1234;                 // PAN ID deines Netzes
static const uint8_t  THREAD_CHANNEL       = 15;                     // Kanal (11..26)
static const uint8_t  THREAD_EXT_PAN_ID[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t  THREAD_NETWORK_KEY[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

// --------------------------------------------------------------------------
// Globale (letzte) Messwerte - werden von loop() aktualisiert und von den
// CoAP-Handlern gelesen.
// --------------------------------------------------------------------------
static volatile float g_temp  = 0.0f;
static volatile float g_press = 0.0f;  // hPa
static volatile float g_hum   = 0.0f;  // %
static volatile float g_wind  = 0.0f;  // m/s
static volatile float g_rain  = 0.0f;  // mm, kumuliert

// --------------------------------------------------------------------------
// Sensoren
// --------------------------------------------------------------------------
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;

static volatile uint32_t rainTicks = 0;  // vom ISR erhöht, in loop() abgetragen

IRAM_ATTR void rainSensorInterrupt() {
  rainTicks++;
}

// --------------------------------------------------------------------------
// Einen Messwert abtasten (BME280 + Anemometer) und mitteln
// --------------------------------------------------------------------------
static void sampleAndAverage(float *t, float *p, float *h, float *w) {
  float st = 0, sp = 0, sh = 0, sw = 0;
  for (int i = 0; i < SAMPLING_COUNT; i++) {
    st += bme.readTemperature();
    sp += bme.readPressure() / 100.0F;
    sh += bme.readHumidity();

    double v = analogRead(ANEMOMETER);
    // ESP32-H2: 12-bit ADC (0-4095), 3.3V Referenz, Vollbereich = 25 m/s
    sw += (float)((v / 4095.0) * 3.3 * 25);

    delay(SAMPLING_DELAY);
  }
  *t = st / SAMPLING_COUNT;
  *p = sp / SAMPLING_COUNT;
  *h = sh / SAMPLING_COUNT;
  *w = sw / SAMPLING_COUNT;
}

// --------------------------------------------------------------------------
// CoAP-Ressourcen
// --------------------------------------------------------------------------
static void buildJson(char *buf, size_t len) {
  snprintf(buf, len,
           "{\"temp\":%.2f,\"pressure\":%.2f,\"humidity\":%.2f,"
           "\"wind\":%.2f,\"rain\":%.2f}",
           (double)g_temp, (double)g_press, (double)g_hum, (double)g_wind, (double)g_rain);
}

static void handleTemp(OThreadCoAPRequest &req, OThreadCoAPResponse &resp, void *ctx) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%.2f", (double)g_temp);
  resp.setContentFormat(OT_COAP_FORMAT_TEXT);
  resp.setPayload(buf);
  resp.send();
}

static void handlePressure(OThreadCoAPRequest &req, OThreadCoAPResponse &resp, void *ctx) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%.2f", (double)g_press);
  resp.setContentFormat(OT_COAP_FORMAT_TEXT);
  resp.setPayload(buf);
  resp.send();
}

static void handleHumidity(OThreadCoAPRequest &req, OThreadCoAPResponse &resp, void *ctx) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%.2f", (double)g_hum);
  resp.setContentFormat(OT_COAP_FORMAT_TEXT);
  resp.setPayload(buf);
  resp.send();
}

static void handleAll(OThreadCoAPRequest &req, OThreadCoAPResponse &resp, void *ctx) {
  char buf[192];
  buildJson(buf, sizeof(buf));
  resp.setContentFormat(OT_COAP_FORMAT_JSON);
  resp.setPayload(buf);
  resp.send();
}

// --------------------------------------------------------------------------
// Thread-Netz beitreten (Dataset-basiert)
// --------------------------------------------------------------------------
static bool joinNetwork() {
  OThread.begin(false);  // kein Auto-Start; wir konfigurieren das Dataset selbst

  DataSet ds;
  ds.setNetworkName(THREAD_NETWORK_NAME);
  ds.setPanId(THREAD_PAN_ID);
  ds.setChannel(THREAD_CHANNEL);
  ds.setExtendedPanId(THREAD_EXT_PAN_ID);
  ds.setNetworkKey(THREAD_NETWORK_KEY);

  OThread.commitDataSet(ds);

  OThread.networkInterfaceUp();
  OThread.start();

  Serial.print("Warte auf Thread-Verbindung");
  uint32_t start = millis();
  while (OThread.otGetDeviceRole() < OT_ROLE_CHILD) {
    if (millis() - start > 30000) {
      Serial.println("\nTimeout - keine Thread-Verbindung.");
      return false;
    }
    Serial.print('.');
    delay(200);
  }
  Serial.println();

  Serial.printf("Verbunden als %s\n", OThread.otGetStringDeviceRole());
  OThread.otPrintNetworkInformation(Serial);
  return true;
}

// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== ESP-BME: Thread + CoAP ===");

  // I2C für BME280
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);

  // Regenmesser (hardware-entprellter Tipping Bucket)
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), rainSensorInterrupt, FALLING);

  // BME280
  if (!bme.begin(0x76, &I2CBME)) {
    Serial.println("BME280 nicht gefunden, verdrahtung prüfen!");
    while (1) delay(1000);
  }

  // Dem Thread-Netz beitreten
  if (!joinNetwork()) {
    Serial.println("Thread-Beitritt fehlgeschlagen. Bitte Dataset-Werte prüfen.");
  }

  // CoAP-Server starten und Ressourcen registrieren.
  // Hinweis: OThreadCoAPServer fasst nur OT_COAP_MAX_RESOURCES (Default 4)
  // Ressourcen. Wind/Regen werden daher nicht als einzelne Endpoints, sondern
  // nur im JSON-Endpoint /weather/all angeboten.
  OThreadCoAPServer.on("weather/temp",     OT_COAP_METHOD_GET, handleTemp,     nullptr);
  OThreadCoAPServer.on("weather/pressure", OT_COAP_METHOD_GET, handlePressure, nullptr);
  OThreadCoAPServer.on("weather/humidity", OT_COAP_METHOD_GET, handleHumidity, nullptr);
  OThreadCoAPServer.on("weather/all",      OT_COAP_METHOD_GET, handleAll,      nullptr);
  OThreadCoAPServer.begin();

  Serial.println("CoAP-Server bereit. Ressourcen: /weather/temp, /weather/pressure, /weather/humidity, /weather/all (JSON)");
}

// --------------------------------------------------------------------------
void loop() {
  static uint32_t lastReport = 0;

  // Nur senden/sampeln, wenn Thread verbunden ist
  if (OThread.otGetDeviceRole() >= OT_ROLE_CHILD) {
    uint32_t now = millis();
    if (now - lastReport >= REPORT_INTERVAL_MS) {
      lastReport = now;

      float t, p, h, w;
      sampleAndAverage(&t, &p, &h, &w);
      g_temp = t;
      g_press = p;
      g_hum = h;
      g_wind = w;

      // Regen-Ticks abtragen und kumulieren
      uint32_t ticks = 0;
      noInterrupts();
      ticks = rainTicks;
      rainTicks = 0;
      interrupts();
      g_rain += ticks * RAIN_MM_PER_TIP;

      Serial.printf("T %.2f | P %.2f | H %.2f | Wind %.2f m/s | Regen %.2f mm\n",
                    (double)t, (double)p, (double)h, (double)w, (double)g_rain);
    }
  } else {
    delay(500);
  }

  delay(50);
}
