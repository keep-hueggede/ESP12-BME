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
#include "SensorStruct.h"

// --------------------------------------------------------------------------
// Hardware-Konfiguration
// --------------------------------------------------------------------------
#define SEALEVELPRESSURE_HPA (1013.25)
#define I2C_SDA 5          // BME280 real an GPIO5 (SDA), ermittelt per Bus-Scan
#define I2C_SCL 4          // BME280 real an GPIO4 (SCL), ermittelt per Bus-Scan
#define BME_ADDR 0x77      // BME280 antwortet auf 0x77 (statt Default 0x76)
#define ANEMOMETER 1 // GPIO1 = ADC1_CH0 am ESP32-H2, analogRead nimmt die Pin-Nummer
#define INTERRUPT_PIN 12    // Regenmesser (Tipping Bucket), Interrupt
#define LED_PIN 8           // Onboard-WS2812B (RGB-LED) - wird beim Start ausgeschaltet

// Abtastung
#define SAMPLING_COUNT 10
#define SAMPLING_DELAY 100
#define REPORT_INTERVAL_MS 5000   // Intervall, in dem die Messwerte aktualisiert werden

// Regenmesser: 1 Kippung = 0.45 mm
#define RAIN_MM_PER_TIP 0.45

// --------------------------------------------------------------------------
// Thread-Netzkonfiguration (Operational Dataset)
//
// Dieses Gerät tritt dem vom ESP-ThreadReceiver (Leader, ESP32-C6) geformten
// Thread-Netzwerk bei. Die Werte sind identisch zu ESP-ThreadReceiver.ino.
// --------------------------------------------------------------------------
static const char    THREAD_NETWORK_NAME[] = "Wetter-Thread";
static const uint16_t THREAD_PAN_ID        = 0x2345;
static const uint8_t  THREAD_CHANNEL       = 15;
static const uint8_t  THREAD_EXT_PAN_ID[8] = {0xDE,0xAD,0xBE,0xEF,0x00,0x00,0x01,0x02};
static const uint8_t  THREAD_NETWORK_KEY[16] = {0xA1,0xB2,0xC3,0xD4,0xE5,0xF6,0x07,0x18,
                                                0x29,0x3A,0x4B,0x5C,0x6D,0x7E,0x8F,0x90};

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

static bme280Struct meanVals;  // zuletzt gemittelte Messwerte (T, P, Alt, Hum, Wind)

IRAM_ATTR void rainSensorInterrupt() {
  rainTicks++;
}

// --------------------------------------------------------------------------
// Einzelnen Messwert vom BME280 + Anemometer lesen
// --------------------------------------------------------------------------
static void readSensors(Adafruit_BME280 *sens, bme280Struct *vals) {
  vals->t = sens->readTemperature();
  vals->p = sens->readPressure() / 100.0F;
  vals->alt = sens->readAltitude(SEALEVELPRESSURE_HPA);
  vals->hum = sens->readHumidity();

  double v = analogRead(ANEMOMETER);
  // ESP32-H2: 12-bit ADC (0-4095), 3.3V Referenz, Vollbereich = 25 m/s
  vals->wSpeed = (v / 4095.0) * 3.3 * 25;
}

// --------------------------------------------------------------------------
// SAMPLING_COUNT-Werte sammeln und mitteln
// --------------------------------------------------------------------------
static void sampleData(bme280Struct *mean) {
  mean->t = 0; mean->p = 0; mean->alt = 0; mean->hum = 0; mean->wSpeed = 0;

  bme280Struct sensorVals[SAMPLING_COUNT];
  for (int i = 0; i < SAMPLING_COUNT; i++) {
    readSensors(&bme, &sensorVals[i]);
    delay(SAMPLING_DELAY);
  }

  for (int i = 0; i < SAMPLING_COUNT; i++) {
    mean->t += sensorVals[i].t;
    mean->p += sensorVals[i].p;
    mean->alt += sensorVals[i].alt;
    mean->hum += sensorVals[i].hum;
    mean->wSpeed += sensorVals[i].wSpeed;
  }
  mean->t /= SAMPLING_COUNT;
  mean->p /= SAMPLING_COUNT;
  mean->alt /= SAMPLING_COUNT;
  mean->hum /= SAMPLING_COUNT;
  mean->wSpeed /= SAMPLING_COUNT;
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
// Thread-Netz beitreten
//
// Das Gerät provisioniert sich mit denselben Netzparametern wie der Receiver
// (ESP32-C6) und tritt dessen Netz bei. Details und die Begründung für
// initNew() stehen im Kommentar direkt an der Dataset-Erzeugung.
//
// OFFEN: Receiver und Sensor erzeugen je ein eigenes initNew()-Dataset und
// damit je ein eigenes Mesh-Local-Prefix. Beide attachen zwar (gemeinsamer
// Network Key), gleichen ihre Datasets aber nicht ab, weil
// otDatasetCreateNewNetwork() beiden denselben Active Timestamp gibt.
// CoAP zwischen den Knoten ist dadurch noch nicht routbar. Saubere Lösung
// ist der Commissioner/Joiner-Pfad (startCommissioner/addJoiner auf dem
// Leader, startJoiner hier), bei dem der Joiner das vollständige Dataset
// des Leaders über die Luft erhält.
// --------------------------------------------------------------------------
static bool joinNetwork() {
  OThread.begin(false);  // kein Auto-Start; wir konfigurieren das Dataset selbst

  // WICHTIG: otDatasetSetActive() lehnt ein Dataset OHNE Active Timestamp ab
  // (OT_ERROR_INVALID_ARGS). Den Timestamp setzt nur initNew(); ein per
  // clear() + Einzelfeldern gebautes Dataset wird still verworfen - der
  // Fehler landet nur in log_e und ist ohne Core-Debug-Level unsichtbar.
  // Der Knoten haette dann gar kein Active Dataset und kann nicht attachen.
  //
  // Deshalb: initNew() (setzt Timestamp, Mesh-Local-Prefix, PSKc) und danach
  // die gemeinsamen Netzparameter ueberschreiben. Extended PAN ID, Key,
  // Channel und PAN ID sind mit dem Leader identisch - die Partitionen
  // verschmelzen, das Dataset mit dem hoeheren Timestamp gewinnt.
  // Nur beim ersten Start; danach gilt das persistierte Dataset aus NVS.
  if (!OThread.hasActiveDataset()) {
    DataSet ds;
    ds.initNew();
    ds.setNetworkName(THREAD_NETWORK_NAME);
    ds.setPanId(THREAD_PAN_ID);
    ds.setChannel(THREAD_CHANNEL);
    ds.setExtendedPanId(THREAD_EXT_PAN_ID);
    ds.setNetworkKey(THREAD_NETWORK_KEY);
    OThread.commitDataSet(ds);
    Serial.println("Neues Dataset angelegt (initNew + Netzparameter).");
  } else {
    Serial.println("Wiederverwende gespeichertes Dataset aus NVS.");
  }

  // Diagnose: was steht nach dem Commit wirklich im Active Dataset?
  Serial.printf("Dataset: ActiveDs=%s Name=%s PAN=0x%04X Ch=%u\n",
                OThread.hasActiveDataset() ? "ja" : "NEIN",
                OThread.getNetworkName().c_str(),
                (int)OThread.getPanId(), (int)OThread.getChannel());

  OThread.networkInterfaceUp();
  OThread.start();

  Serial.print("Warte auf Thread-Verbindung");
  uint32_t start = millis();
  while (OThread.otGetDeviceRole() < OT_ROLE_CHILD) {
    if (millis() - start > 60000) {
      Serial.println("\nTimeout - keine Thread-Verbindung.");
      return false;
    }
    Serial.print('.');
    delay(200);
  }
  Serial.println();

  Serial.printf("Verbunden als %s\n", OThread.otGetStringDeviceRole());
  OThread.otPrintNetworkInformation(Serial);
  // Mesh-Local EID für den ESP-ThreadReceiver (nodes[] im Receiver-Sketch)
  Serial.printf("Mesh-Local EID: %s\n", OThread.getMeshLocalEid().toString().c_str());
  return true;
}

// --------------------------------------------------------------------------
// I²C-Bus scannen und gefundene Adressen ausgeben (Diagnose im Fehlerfall)
// --------------------------------------------------------------------------
static void scanI2C() {
  Serial.println("I²C-Scan (SDA=" + String(I2C_SDA) + ", SCL=" + String(I2C_SCL) + "):");
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    I2CBME.beginTransmission(address);
    error = I2CBME.endTransmission();
    if (error == 0) {
      Serial.print("  Gerät gefunden bei 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      nDevices++;
    }
  }
  Serial.printf("  %d Gerät(e) gefunden.\n", nDevices);
}

// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== ESP-BME: Thread + CoAP ===");

  // Onboard-WS2812B-LED abschalten (GPIO8, im Projekt ungenutzt)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // I2C für BME280 (Pins/Adresse laut Schaltplan realer Verdrahtung)
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);

  // Regenmesser (hardware-entprellter Tipping Bucket)
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), rainSensorInterrupt, FALLING);

  // BME280
  if (!bme.begin(BME_ADDR, &I2CBME)) {
    Serial.println("BME280 nicht gefunden, verdrahtung prüfen!");
    scanI2C();
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
// Gemittelte Messwerte in einer uebersichtlichen Tabelle auf dem Terminal
// ausgeben (ASCII-only wg. Serial-Konsistenz).
// --------------------------------------------------------------------------
static void printValues(const bme280Struct *v, float rain) {
  uint32_t ms = millis();
  uint32_t h = ms / 3600000UL;
  uint32_t m = (ms / 60000UL) % 60;
  uint32_t s = (ms / 1000UL) % 60;

  Serial.println();
  Serial.println("+-------------------+------------------+");
  Serial.printf  ("| Laufzeit          | %02lu:%02lu:%02lu          |\n", h, m, s);
  Serial.println("+-------------------+------------------+");
  Serial.printf  ("| Temperatur        | %6.2f C        |\n", v->t);
  Serial.printf  ("| Luftdruck         | %7.2f hPa     |\n", v->p);
  Serial.printf  ("| Hoehe (barometr.) | %7.2f m       |\n", v->alt);
  Serial.printf  ("| Luftfeuchte       | %6.2f %        |\n", v->hum);
  Serial.printf  ("| Windgeschwindigkeit| %5.2f m/s     |\n", v->wSpeed);
  Serial.printf  ("| Regen (kumulativ) | %7.2f mm      |\n", rain);
  Serial.println("+-------------------+------------------+");
}

// --------------------------------------------------------------------------
void loop() {
  static uint32_t lastReport = 0;

  uint32_t now = millis();
  if (now - lastReport >= REPORT_INTERVAL_MS) {
    lastReport = now;

    sampleData(&meanVals);
    g_temp = (float)meanVals.t;
    g_press = (float)meanVals.p;
    g_hum = (float)meanVals.hum;
    g_wind = (float)meanVals.wSpeed;

    // Regen-Ticks abtragen und kumulieren
    uint32_t ticks = 0;
    noInterrupts();
    ticks = rainTicks;
    rainTicks = 0;
    interrupts();
    g_rain += ticks * RAIN_MM_PER_TIP;

    printValues(&meanVals, g_rain);
  }

  delay(50);
}
