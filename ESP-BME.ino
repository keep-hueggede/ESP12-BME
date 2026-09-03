/******************************************
 * ESP-BME - Wetter-Sensorknoten (Thread Sleepy End Device)
 *
 * ESP32-H2 (ESP32-H2FH4S) Wetterknoten. Er misst Temperatur, Luftdruck,
 * Luftfeuchte, Windgeschwindigkeit und Regen und SENDET die Werte im festen
 * Takt per CoAP an den Collector (ESP-ThreadReceiver, ESP32-C6).
 *
 * Rolle:          Thread Sleepy End Device (SED), CoAP-Client
 *
 * Warum senden statt abfragen:
 *                 Als SED ist das Funkmodul im Leerlauf ABGESCHALTET. Eine
 *                 eingehende Abfrage landet beim Parent, wird dort gepuffert
 *                 und erst beim naechsten Data Poll zugestellt - bei einem
 *                 stromsparenden Poll-Intervall laeuft jede Abfrage in den
 *                 Timeout. Ausserdem ist Empfangsbereitschaft der teure Teil,
 *                 nicht das Senden. Deshalb gibt es hier KEINEN CoAP-Server
 *                 mehr; der Knoten meldet sich von sich aus.
 *
 * Netzbeitritt:   Per Thread-Commissioning. Der Knoten kennt nur den PSKd
 *                 (siehe JOINER_PSKD) und erhaelt das vollstaendige Dataset
 *                 - inklusive Network Key und Mesh-Local-Prefix - vom
 *                 Commissioner ueber die Luft. Es steht bewusst KEIN
 *                 Network Key und kein Mesh-Local-Prefix in diesem Sketch:
 *                 haette jeder Knoten sein eigenes per initNew() erzeugtes
 *                 Dataset, haette er auch ein eigenes Mesh-Local-Prefix und
 *                 waere fuer die anderen nicht routbar.
 *
 * Neuer Sensor:   Nur SENSOR_NAME anpassen und flashen. Der Collector
 *                 braucht keine Aenderung.
 *
 * Benoetigt den Arduino-Core (arduino-esp32 v3.x) mit OpenThread-Unterstuetzung
 * und CONFIG_OPENTHREAD_JOINER=y (im Stock-Core gesetzt).
 *******************************************/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OThread.h>
#include <OThreadCoAP.h>
#include "SensorStruct.h"

// Rohe OpenThread-Aufrufe: der Arduino-Wrapper exponiert fuer SED nur
// getPollPeriod() (einen Getter), die Konfiguration gibt es nur nativ.
#include <openthread/thread.h>
#include <openthread/link.h>
#include "esp_openthread_lock.h"

// --------------------------------------------------------------------------
// Identitaet dieses Knotens
//
// Wird als Feld "src" mitgesendet und ist das einzige, was sich bei einem
// weiteren Sensor derselben Bauart aendern muss.
// --------------------------------------------------------------------------
static const char SENSOR_NAME[] = "wetter1";

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

// Abtastung: SAMPLING_COUNT Messungen im Abstand SAMPLING_DELAY werden
// gemittelt. Das kostet SAMPLING_COUNT * SAMPLING_DELAY ms Wachzeit pro Zyklus.
#define SAMPLING_COUNT 10
#define SAMPLING_DELAY 100

// Regenmesser: 1 Kippung = 0.45 mm
#define RAIN_MM_PER_TIP 0.45

// --------------------------------------------------------------------------
// Sende- und Polltakt
// --------------------------------------------------------------------------
#define REPORT_INTERVAL_MS   60000   // alle 60 s messen und senden

// Data-Poll-Intervall des SED: so oft fragt der Knoten seinen Parent nach
// gepufferten Nachrichten. Laenger = sparsamer, aber traeger fuer alles, was
// von aussen an den Knoten geschickt wird. Da wir nur senden, darf es lang
// sein; das ACK auf unser POST holt der Stack unmittelbar nach dem Senden ab.
#define SED_POLL_PERIOD_MS   60000

// Waehrend eines Sendevorgangs wird kurzzeitig schnell gepollt. Grund: die
// Antwort auf ein CON-Request liegt beim Parent gepuffert und wird erst beim
// naechsten Data Poll zugestellt. Mit dem langen Ruhe-Intervall kaeme das ACK
// lange nach COAP_TIMEOUT_MS an, und jeder Push liefe in den Timeout.
// Das Fenster kostet nur wenige hundert ms Funkzeit pro Minute.
#define SED_TX_POLL_PERIOD_MS  250

// Der Parent verwirft ein Kind, das sich SED_CHILD_TIMEOUT_S lang nicht
// meldet. Muss deutlich ueber dem Poll-Intervall liegen.
#define SED_CHILD_TIMEOUT_S  240

// --------------------------------------------------------------------------
// Commissioning
//
// Muss mit JOINER_PSKD im ESP-ThreadReceiver uebereinstimmen. Der Collector
// oeffnet das Anmeldefenster beim Start bzw. auf Tastendruck 'c'.
// --------------------------------------------------------------------------
static const char JOINER_PSKD[] = "WETTER01";
#define JOINER_TIMEOUT_MS    60000   // wie lange ein Beitrittsversuch laeuft
#define JOINER_RETRIES       3       // Versuche, bevor aufgegeben wird

// --------------------------------------------------------------------------
// Ziel: CoAP-Ressource des Collectors
//
// Die Adresse wird NICHT hartcodiert, sondern zur Laufzeit aus der Rolle
// abgeleitet: getLeaderRloc() liefert den Leader im eigenen Mesh-Local-
// Prefix. Damit ueberlebt der Sensor jeden Prefix-Wechsel und jeden Tausch
// des Collectors.
// --------------------------------------------------------------------------
static const char *INGEST_PATH = "ingest";
#define COAP_TIMEOUT_MS  5000

// --------------------------------------------------------------------------
// Messwerte
// --------------------------------------------------------------------------
static float g_temp  = 0.0f;
static float g_press = 0.0f;  // hPa
static float g_alt   = 0.0f;  // m
static float g_hum   = 0.0f;  // %
static float g_wind  = 0.0f;  // m/s
static float g_rain  = 0.0f;  // mm, kumuliert

// Statistik
static uint32_t g_sent = 0;
static uint32_t g_failed = 0;

// --------------------------------------------------------------------------
// Sensoren
// --------------------------------------------------------------------------
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;
OThreadCoAPClient coap;

static volatile uint32_t rainTicks = 0;  // vom ISR erhoeht, in loop() abgetragen

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
// Nutzlast bauen: ein vollstaendiges JSON-Objekt inklusive "src". Der
// Collector reicht die Zeile unveraendert an den Pi weiter.
// --------------------------------------------------------------------------
static void buildJson(char *buf, size_t len) {
  snprintf(buf, len,
           "{\"src\":\"%s\",\"temp\":%.2f,\"pressure\":%.2f,\"altitude\":%.2f,"
           "\"humidity\":%.2f,\"wind\":%.2f,\"rain\":%.2f}",
           SENSOR_NAME, (double)g_temp, (double)g_press, (double)g_alt,
           (double)g_hum, (double)g_wind, (double)g_rain);
}

// --------------------------------------------------------------------------
// Als Sleepy End Device konfigurieren.
//
// mRxOnWhenIdle = false ist die eigentliche Ersparnis: das Funkmodul bleibt
// zwischen den Data Polls aus. mDeviceType = false meldet den Knoten als MTD
// an - er wird nie Router und nie Parent fuer andere Knoten.
//
// Muss VOR OThread.start() aufgerufen werden, damit der Knoten von vornherein
// als SED attached und nicht erst als Router auftritt.
// --------------------------------------------------------------------------
static bool configureSleepyEndDevice() {
  otInstance *inst = OThread.getInstance();
  if (!inst) {
    Serial.println("Keine OpenThread-Instanz - SED-Konfiguration uebersprungen.");
    return false;
  }

  otLinkModeConfig mode;
  mode.mRxOnWhenIdle = false;  // Funkmodul im Leerlauf aus
  mode.mDeviceType   = false;  // MTD: kein Router, kein Parent
  mode.mNetworkData  = false;  // nur stabile Netzdaten anfordern

  bool ok = false;
  if (esp_openthread_lock_acquire(portMAX_DELAY)) {
    otError e1 = otThreadSetLinkMode(inst, mode);
    otError e2 = otLinkSetPollPeriod(inst, SED_POLL_PERIOD_MS);
    otThreadSetChildTimeout(inst, SED_CHILD_TIMEOUT_S);
    ok = (e1 == OT_ERROR_NONE && e2 == OT_ERROR_NONE);
    esp_openthread_lock_release();
    if (!ok) {
      Serial.printf("SED-Konfiguration fehlgeschlagen: LinkMode=%d PollPeriod=%d\n", (int)e1, (int)e2);
    }
  } else {
    Serial.println("OpenThread-Lock nicht bekommen - SED-Konfiguration uebersprungen.");
  }
  return ok;
}

// --------------------------------------------------------------------------
// Data-Poll-Intervall zur Laufzeit umschalten.
// --------------------------------------------------------------------------
static void setPollPeriod(uint32_t ms) {
  otInstance *inst = OThread.getInstance();
  if (!inst) return;
  if (esp_openthread_lock_acquire(portMAX_DELAY)) {
    otLinkSetPollPeriod(inst, ms);
    esp_openthread_lock_release();
  }
}

// --------------------------------------------------------------------------
// Dem Thread-Netz beitreten.
//
// Beim ersten Start per Commissioning (Joiner): der Knoten erhaelt das
// vollstaendige Dataset vom Commissioner. Danach liegt es im NVS und wird
// wiederverwendet - ein Reboot braucht kein offenes Anmeldefenster mehr.
// --------------------------------------------------------------------------
static bool joinNetwork() {
  OThread.begin(false);            // kein Auto-Start; wir steuern die Reihenfolge
  OThread.networkInterfaceUp();    // startJoiner() braucht das IPv6-Interface

  if (!OThread.hasActiveDataset()) {
#if CONFIG_OPENTHREAD_JOINER
    Serial.println("Kein Dataset im NVS - starte Commissioning (Joiner).");
    Serial.println("Am Collector muss dafuer ein Anmeldefenster offen sein ('c' in dessen Konsole).");

    bool joined = false;
    for (int attempt = 1; attempt <= JOINER_RETRIES && !joined; attempt++) {
      Serial.printf("  Beitrittsversuch %d/%d ...\n", attempt, JOINER_RETRIES);
      otError err = OThread.startJoiner(JOINER_PSKD, JOINER_TIMEOUT_MS);
      if (err == OT_ERROR_NONE) {
        joined = true;
      } else {
        Serial.printf("  fehlgeschlagen (%d)\n", (int)err);
      }
    }

    if (!joined) {
      Serial.println("Commissioning fehlgeschlagen - kein Dataset erhalten.");
      return false;
    }
    Serial.println("Commissioning erfolgreich, Dataset erhalten.");
#else
    Serial.println("CONFIG_OPENTHREAD_JOINER ist im Core-Build nicht aktiviert.");
    return false;
#endif
  } else {
    Serial.println("Wiederverwende gespeichertes Dataset aus NVS.");
  }

  // SED-Modus setzen, BEVOR Thread startet.
  configureSleepyEndDevice();

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
  Serial.printf("Mesh-Local EID: %s\n", OThread.getMeshLocalEid().toString().c_str());
  Serial.printf("Poll-Intervall: %lu ms\n", (unsigned long)OThread.getPollPeriod());
  return true;
}

// --------------------------------------------------------------------------
// Eine Messung an den Collector schicken.
// Rueckgabe: true, wenn der Collector quittiert hat.
// --------------------------------------------------------------------------
static bool pushMeasurement() {
  if (OThread.otGetDeviceRole() < OT_ROLE_CHILD) {
    Serial.println("Nicht attached - Messung wird verworfen.");
    g_failed++;
    return false;
  }

  IPAddress collector = OThread.getLeaderRloc();

  char json[224];
  buildJson(json, sizeof(json));

  // Schnell pollen, solange wir auf das ACK warten - danach wieder schlafen.
  setPollPeriod(SED_TX_POLL_PERIOD_MS);
  int code = coap.POST(collector, INGEST_PATH, json);
  setPollPeriod(SED_POLL_PERIOD_MS);

  if (code > 0) {
    g_sent++;
    Serial.printf("TX   -> %s  code=%d  %s\n", collector.toString().c_str(), code, json);
    return true;
  }

  g_failed++;
  Serial.printf("FAIL -> %s  code=%d (gesendet=%lu fehlgeschlagen=%lu)\n",
                collector.toString().c_str(), code,
                (unsigned long)g_sent, (unsigned long)g_failed);
  return false;
}

// --------------------------------------------------------------------------
// I2C-Bus scannen und gefundene Adressen ausgeben (Diagnose im Fehlerfall)
// --------------------------------------------------------------------------
static void scanI2C() {
  Serial.println("I2C-Scan (SDA=" + String(I2C_SDA) + ", SCL=" + String(I2C_SCL) + "):");
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    I2CBME.beginTransmission(address);
    error = I2CBME.endTransmission();
    if (error == 0) {
      Serial.print("  Geraet gefunden bei 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      nDevices++;
    }
  }
  Serial.printf("  %d Geraet(e) gefunden.\n", nDevices);
}

// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("=== ESP-BME '%s': Thread SED + CoAP-Push ===\n", SENSOR_NAME);

  // Onboard-WS2812B-LED abschalten (GPIO8, im Projekt ungenutzt)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // I2C fuer BME280 (Pins/Adresse laut realer Verdrahtung)
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);

  // Regenmesser (hardware-entprellter Tipping Bucket)
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), rainSensorInterrupt, FALLING);

  // BME280
  if (!bme.begin(BME_ADDR, &I2CBME)) {
    Serial.println("BME280 nicht gefunden, Verdrahtung pruefen!");
    scanI2C();
    while (1) delay(1000);
  }

  coap.setTimeout(COAP_TIMEOUT_MS);

  if (!joinNetwork()) {
    Serial.println("Thread-Beitritt fehlgeschlagen.");
  }

  Serial.printf("Bereit. Sende alle %d ms an /%s des Collectors.\n", REPORT_INTERVAL_MS, INGEST_PATH);
}

// --------------------------------------------------------------------------
void loop() {
  static uint32_t lastReport = 0;
  static bool first = true;

  uint32_t now = millis();
  if (first || now - lastReport >= REPORT_INTERVAL_MS) {
    first = false;
    lastReport = now;

    sampleData(&meanVals);
    g_temp  = (float)meanVals.t;
    g_press = (float)meanVals.p;
    g_alt   = (float)meanVals.alt;
    g_hum   = (float)meanVals.hum;
    g_wind  = (float)meanVals.wSpeed;

    // Regen-Ticks abtragen und kumulieren
    uint32_t ticks = 0;
    noInterrupts();
    ticks = rainTicks;
    rainTicks = 0;
    interrupts();
    g_rain += ticks * RAIN_MM_PER_TIP;

    pushMeasurement();
  }

  delay(100);
}
