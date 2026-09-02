/******************************************
 * ESP-BME-LoRa - Wetterstation über LoRa
 *
 * ESP32-H2 (ESP32-H2FH4S) Wetterknoten für den Einsatz im Wald.
 * Messwerte (BME280, Anemometer, Regen) werden periodisch per LoRa
 * (SX1276/77/78/79, 868 MHz EU) an die zentrale Empfangsstation
 * (ESP-ThreadReceiver mit LoRa-Modul) gesendet.
 *
 * Übertragung:    LoRa (SPI, SX127x) @ 868 MHz, SF12 für maximale Reichweite
 * Rolle:          LoRa-Sender (Waldknoten), sendet JSON-Pakete
 *
 * Benötigt: RadioLib (arduino-cli lib install RadioLib) und ein
 * LoRa-Modul am SPI-Bus des ESP32-H2.
 *******************************************/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RadioLib.h>
#include "SensorStruct.h"

// --------------------------------------------------------------------------
// Hardware-Konfiguration
// --------------------------------------------------------------------------
#define SEALEVELPRESSURE_HPA (1013.25)
#define I2C_SDA 5          // BME280 real an GPIO5 (SDA), ermittelt per Bus-Scan
#define I2C_SCL 4          // BME280 real an GPIO4 (SCL), ermittelt per Bus-Scan
#define BME_ADDR 0x77      // BME280 antwortet auf 0x77 (statt Default 0x76)
#define ANEMOMETER 1       // GPIO1 = ADC1_CH0 am ESP32-H2, analogRead nimmt die Pin-Nummer
#define INTERRUPT_PIN 12   // Regenmesser (Tipping Bucket), Interrupt
#define LED_PIN 8          // Onboard-WS2812B (RGB-LED) - wird beim Start ausgeschaltet

// LoRa-Modul am SPI-Bus des ESP32-H2 (Verdrahtung anpassen!)
#define LORA_SS   0        // NSS/CS  -> GPIO0 (Default SS des H2)
#define LORA_RST  2        // Reset   -> GPIO2 (frei)
#define LORA_DIO0 3        // DIO0    -> GPIO3 (frei, optional für RX-Interrupts)
// SCK=GPIO10, MOSI=GPIO25, MISO=GPIO11 (Default-SPI des H2)

// Energieoptimierung: Takt reduzieren. 48 MHz spart Arbeitssstrom; der LoRa
// Transceiver läuft unabhängig vom CPU-Takt.
#define CPU_FREQ_MHZ 48

// Abtastung: Wenige Samples reichen; je geringer, desto seltener muss die
// CPU aufwachen und der BME messen.
#define SAMPLING_COUNT 3
#define SAMPLING_DELAY 5
#define SEND_INTERVAL_MS 60000   // Sendeintervall im Wald (60 s; Batterie-freundlich)

// Regenmesser: 1 Kippung = 0.45 mm
#define RAIN_MM_PER_TIP 0.45

// --------------------------------------------------------------------------
// LoRa-Funkparameter (müssen auf Sender und Empfänger identisch sein!)
// --------------------------------------------------------------------------
#define LORA_FREQ_MHZ   868.1F   // EU-LoRa ISM-Band, Kanal 1
#define LORA_SF         12       // Spreading Factor 12 = maximale Reichweite
#define LORA_BW         125.0F   // Bandbreite 125 kHz
#define LORA_CR         8        // Coding Rate 4/8 (mehr Redundanz, bessere Link-Budget)
#define LORA_SYNC       0x12     // Sync-Word (Standard LoRa)
#define LORA_POWER      20       // Sendeleistung in dBm (max. ~20 bei SX127x PA_BOOST)
#define LORA_PREAMBLE   8        // Preamble-Symbole
#define LORA_TX_TIMEOUT 5000     // RadioLib-Timeout für einen Sendevorgang (ms)

// Knotenbezeichnung (geht als Prefix ins LoRa-Paket, damit die Zentrale
// mehrere Waldknoten unterscheiden kann)
static const char NODE_NAME[] = "wetter1";

// --------------------------------------------------------------------------
// Globale (letzte) Messwerte - werden von loop() aktualisiert
// --------------------------------------------------------------------------
static volatile float g_temp  = 0.0f;
static volatile float g_press = 0.0f;  // hPa
static volatile float g_hum   = 0.0f;  // %
static volatile float g_wind  = 0.0f;  // m/s
static volatile float g_rain  = 0.0f;  // mm, kumuliert

// --------------------------------------------------------------------------
// Sensoren & Funk
// --------------------------------------------------------------------------
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;
SX1278 radio = new Module((uint32_t)LORA_SS, (uint32_t)LORA_DIO0,
                           (uint32_t)LORA_RST, (uint32_t)RADIOLIB_NC);

static volatile uint32_t rainTicks = 0;  // vom ISR erhöht, in loop() abgetragen

static bme280Struct meanVals;  // zuletzt gemittelte Messwerte (T, P, Alt, Hum, Wind)

IRAM_ATTR void rainSensorInterrupt() {
  rainTicks++;
}

// --------------------------------------------------------------------------
// Einzelnen Messwert vom BME280 + Anemometer lesen
//
// Energiesparmodus: Der BME280 läuft im Forced-Mode. Er schläft dann zwischen
// den Messungen (nur wenige µA) und nimmt auf takeForcedMeasurement() genau
// eine Messung vor.
// --------------------------------------------------------------------------
static void readSensors(Adafruit_BME280 *sens, bme280Struct *vals) {
  sens->takeForcedMeasurement();

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
// JSON-Payload bauen (gleiches Format wie bisher vom CoAP-Server)
// --------------------------------------------------------------------------
static void buildJson(char *buf, size_t len) {
  snprintf(buf, len,
           "{\"temp\":%.2f,\"pressure\":%.2f,\"humidity\":%.2f,"
           "\"wind\":%.2f,\"rain\":%.2f}",
           (double)g_temp, (double)g_press, (double)g_hum, (double)g_wind, (double)g_rain);
}

// --------------------------------------------------------------------------
// LoRa am ESP32-H2 initialisieren
// --------------------------------------------------------------------------
static bool initLora() {
  Serial.println("Initialisiere LoRa-Modul...");
  int state = radio.begin(LORA_FREQ_MHZ, LORA_BW, LORA_SF, LORA_CR,
                          LORA_SYNC, LORA_POWER, LORA_PREAMBLE, LORA_TX_TIMEOUT);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa-Fehler bei begin(): %d. Verdrahtung prüfen!\n", state);
    return false;
  }

  Serial.printf("LoRa bereit: %.1f MHz, SF%d, BW%.0f kHz, CR 4/%d, +%d dBm\n",
                LORA_FREQ_MHZ, (int)LORA_SF, LORA_BW, (int)LORA_CR, (int)LORA_POWER);
  return true;
}

// --------------------------------------------------------------------------
// Aktuelle Messwerte per LoRa an die Zentrale senden.
// Paketformat: "<Knotenname> <JSON>"
// --------------------------------------------------------------------------
static void sendLora() {
  char json[192];
  buildJson(json, sizeof(json));

  char packet[192];
  snprintf(packet, sizeof(packet), "%s %s", NODE_NAME, json);

  Serial.printf("Sende: %s\n", packet);

  int state = radio.transmit((const char *)packet);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("Gesendet (%lu ms on-air).\n",
                  (unsigned long)radio.getTimeOnAir(sizeof(packet)));
  } else {
    Serial.printf("Sendefehler: %d\n", state);
  }
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
  // Energiesparmodus: Takt früh senken (vor I2C/ADC/Funk).
  if (!setCpuFrequencyMhz(CPU_FREQ_MHZ)) {
    Serial.println("CPU-Frequenz konnte nicht gesetzt werden!");
  }

  Serial.begin(115200);
  delay(500);
  Serial.println("=== ESP-BME-LoRa: Waldknoten ===");
  Serial.printf("CPU-Takt: %u MHz\n", (unsigned)getCpuFrequencyMhz());

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

  // Energiesparmodus BME280: Forced-Mode + leichtes Oversampling.
  bme.setSampling(Adafruit_BME280::MODE_FORCED,   // nur messen, wenn angefordert
                  Adafruit_BME280::SAMPLING_X2,   // Temperatur
                  Adafruit_BME280::SAMPLING_X1,   // Druck
                  Adafruit_BME280::SAMPLING_X2,   // Feuchte
                  Adafruit_BME280::FILTER_OFF,    // kein IIR-Filter (spart Umsetzung)
                  Adafruit_BME280::STANDBY_MS_0_5);  // Standby (bei Forced irrelevant)

  // LoRa-Modul starten
  if (!initLora()) {
    Serial.println("LoRa-Modul nicht erreichbar. Bitte Verdrahtung/Spannung prüfen.");
  }
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
  static uint32_t lastSend = 0;

  uint32_t now = millis();
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;

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
    sendLora();
  }

  delay(50);
}