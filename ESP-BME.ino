/******************************************
 * ESP-BME - Wetter-Sensorknoten (Thread Sleepy End Device, Deep Sleep)
 *
 * ESP32-H2 (ESP32-H2FH4S) Wetterknoten. Er misst Temperatur, Luftdruck,
 * Luftfeuchte, Windgeschwindigkeit und Regen und SENDET die Werte im festen
 * Takt per CoAP an den Collector (ESP-ThreadReceiver, ESP32-C6).
 *
 * Rolle:          Thread Sleepy End Device (SED), CoAP-Client
 *
 * Energie:        Der Knoten verbringt fast die ganze Zeit im Deep Sleep.
 *                 Er wacht nur auf, um
 *                  - eine Kippung des Regenmessers zu zaehlen (GPIO-Wake,
 *                    ohne Funk, wenige ms), oder
 *                  - alle REPORT_INTERVAL_S zu messen und zu senden.
 *                 Gesendet wird die Regenmenge SEIT DER LETZTEN erfolgreich
 *                 quittierten Meldung; der Pi summiert. Nicht quittierte
 *                 Kippungen bleiben im RTC-Speicher und gehen mit der
 *                 naechsten Meldung raus.
 *
 * Warum senden statt abfragen:
 *                 Als SED ist das Funkmodul im Leerlauf ABGESCHALTET. Eine
 *                 Abfrage von aussen liefe in den Timeout. Deshalb gibt es
 *                 KEINEN CoAP-Server; der Knoten meldet sich von sich aus.
 *
 * Netzbeitritt:   Per Thread-Commissioning. Der Knoten kennt nur den PSKd
 *                 (siehe JOINER_PSKD) und erhaelt das vollstaendige Dataset
 *                 vom Commissioner ueber die Luft. Danach liegt es im NVS.
 *
 * Identitaet:     Der Knoten sendet seine eindeutige Hardware-ID (EUI-64)
 *                 als "src". Den Anzeigenamen vergibt man in der Web-App
 *                 auf dem Pi - jeder Sensor laeuft mit identischer Firmware.
 *
 * Flashen:        Im Deep Sleep ist der USB-Port weg. Nach RESET bleibt der
 *                 Knoten STARTUP_AWAKE_S wach - in diesem Fenster flashen.
 *
 * Benoetigt den Arduino-Core (arduino-esp32 v3.x) mit OpenThread-Unterstuetzung
 * und CONFIG_OPENTHREAD_JOINER=y (im Stock-Core gesetzt).
 *******************************************/
#include <Arduino.h>
#include <Wire.h>
#include <sys/time.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OThread.h>
#include <OThreadCoAP.h>

// Rohe OpenThread-Aufrufe: der Arduino-Wrapper exponiert fuer SED nur
// getPollPeriod() (einen Getter), die Konfiguration gibt es nur nativ.
#include <openthread/thread.h>
#include <openthread/link.h>
#include "esp_openthread_lock.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_sleep.h"

// --------------------------------------------------------------------------
// Hardware-Konfiguration
// --------------------------------------------------------------------------
#define SEALEVELPRESSURE_HPA (1013.25)
#define I2C_SDA 5          // BME280 real an GPIO5 (SDA), ermittelt per Bus-Scan
#define I2C_SCL 4          // BME280 real an GPIO4 (SCL), ermittelt per Bus-Scan
#define BME_ADDR 0x77      // BME280 antwortet auf 0x77 (statt Default 0x76)
#define ANEMOMETER 1       // GPIO1 = ADC1_CH0 am ESP32-H2, analogRead nimmt die Pin-Nummer
#define RAIN_PIN 12        // Regenmesser (Wippe: Seite A = 3 V, Seite B = 0 V); GPIO7-14 koennen wecken
#define LED_PIN 8          // Onboard-WS2812B (RGB-LED) - wird bei jedem Mess-Zyklus schwarz gesetzt

// Regenmesser: 1 Kippung = 0.45 mm. Jeder stabile Pegelwechsel ist eine Kippung.
#define RAIN_MM_PER_TIP 0.45
#define RAIN_STABLE_MS  100   // level must hold this long to count (filters glitches)

// Batterie ueber Spannungsteiler (2x 1 MOhm + 100 nF) an BATTERY_PIN.
// Erst auf 1 setzen, wenn der Teiler verdrahtet ist - ein offener Pin liefert Unsinn.
#define BATTERY_ENABLED  0
#define BATTERY_PIN      2
#define BATTERY_DIVIDER  2.0f
#define BATTERY_LOW_V    2.3f   // 2x NiMH: ~1.15 V/Zelle -> seltener senden

// --------------------------------------------------------------------------
// Takt
// --------------------------------------------------------------------------
#ifndef REPORT_INTERVAL_S
#define REPORT_INTERVAL_S      300    // alle 5 min messen und senden
#endif
#define LOW_BATTERY_INTERVAL_S 1800   // bei schwacher Batterie alle 30 min
#define STARTUP_AWAKE_S        30     // nach RESET wach bleiben (Konsole, Flashen)
#define ATTACH_TIMEOUT_MS      15000  // max. Wartezeit auf die Thread-Verbindung

// Data-Poll-Intervall waehrend der kurzen Wachphase. Waehrend des Sendens wird
// schnell gepollt, damit das ACK auf das CON-Request zeitnah ankommt.
#define SED_POLL_PERIOD_MS     60000
#define SED_TX_POLL_PERIOD_MS  250

// Der Parent verwirft ein Kind, das sich so lange nicht meldet. Muss ueber dem
// laengsten Schlafintervall liegen, sonst ist nach jedem Aufwachen ein
// kompletter Neu-Beitritt noetig (deutlich mehr Funkzeit).
#define SED_CHILD_TIMEOUT_S    (2 * LOW_BATTERY_INTERVAL_S)

// --------------------------------------------------------------------------
// Commissioning
//
// Muss mit JOINER_PSKD im ESP-ThreadReceiver uebereinstimmen. Der Collector
// oeffnet das Anmeldefenster beim Start bzw. auf Tastendruck 'c'.
// --------------------------------------------------------------------------
static const char JOINER_PSKD[] = "WETTER01";
#define JOINER_TIMEOUT_MS    60000

// --------------------------------------------------------------------------
// Ziel: CoAP-Ressource des Collectors (Leader im eigenen Mesh-Local-Prefix)
// --------------------------------------------------------------------------
static const char *INGEST_PATH = "ingest";
#define COAP_TIMEOUT_MS  5000

// --------------------------------------------------------------------------
// Zustand, der den Deep Sleep ueberlebt (RTC-Speicher)
// --------------------------------------------------------------------------
RTC_DATA_ATTR static volatile int8_t   rtcRainLevel = -1;   // last stable level
RTC_DATA_ATTR static volatile uint32_t rtcTipsPending = 0;  // not yet acknowledged
RTC_DATA_ATTR static int64_t           rtcNextReportUs = 0;
RTC_DATA_ATTR static uint32_t          rtcSent = 0;
RTC_DATA_ATTR static uint32_t          rtcFailed = 0;

// --------------------------------------------------------------------------
// Messwerte dieser Wachphase
// --------------------------------------------------------------------------
static bool  g_bmeOk = false;
static float g_temp, g_press, g_alt, g_hum;
static float g_wind = 0.0f;     // m/s
static float g_battery = NAN;   // V
static char  g_hwId[17] = "unknown";

TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;
OThreadCoAPClient coap;
static esp_timer_handle_t rainTimer = nullptr;
static bool g_stayAwake = false;
static uint32_t g_awakeUntilMs = 0;

// --------------------------------------------------------------------------
// Hilfsfunktionen
// --------------------------------------------------------------------------
// Wall-clock time keeps running in deep sleep (RTC timer), millis() does not
static int64_t nowUs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
}

static void readHardwareId() {
  uint8_t mac[8];
  if (esp_read_mac(mac, ESP_MAC_IEEE802154) == ESP_OK) {
    for (int i = 0; i < 8; i++) snprintf(g_hwId + 2 * i, 3, "%02x", mac[i]);
  }
}

// Wait until the rain pin holds one level for RAIN_STABLE_MS (max. 1 s)
static int stableRainLevel() {
  int level = digitalRead(RAIN_PIN);
  uint32_t since = millis(), start = since;
  while (millis() - since < RAIN_STABLE_MS && millis() - start < 1000) {
    delay(10);
    int now = digitalRead(RAIN_PIN);
    if (now != level) { level = now; since = millis(); }
  }
  return level;
}

static void countRainLevel(int level) {
  if (rtcRainLevel >= 0 && level != rtcRainLevel) rtcTipsPending++;
  rtcRainLevel = level;
}

// While awake the pin is sampled every 20 ms; same stability rule as above
static void rainSample(void *) {
  static int candidate = -1, stable = 0;
  int level = digitalRead(RAIN_PIN);
  if (level != candidate) { candidate = level; stable = 0; return; }
  if (stable < RAIN_STABLE_MS / 20) stable++;
  if (stable == RAIN_STABLE_MS / 20 && candidate != rtcRainLevel) {
    uint32_t before = rtcTipsPending;
    countRainLevel(candidate);
    if (rtcTipsPending != before) Serial.printf("Kippung erkannt (offen: %lu)\n", (unsigned long)rtcTipsPending);
  }
}

static uint32_t reportIntervalS() {
  if (!isnan(g_battery) && g_battery < BATTERY_LOW_V) return LOW_BATTERY_INTERVAL_S;
  return REPORT_INTERVAL_S;
}

// --------------------------------------------------------------------------
// Deep Sleep bis zur naechsten Meldung oder bis zur naechsten Kippung
// --------------------------------------------------------------------------
static void goToSleep() {
  if (rainTimer) esp_timer_stop(rainTimer);

  // Wake on the opposite of the last counted level. If the pin already changed,
  // the chip wakes right away and counts it.
  esp_sleep_enable_ext1_wakeup_io(1ULL << RAIN_PIN,
                                  rtcRainLevel ? ESP_EXT1_WAKEUP_ANY_LOW : ESP_EXT1_WAKEUP_ANY_HIGH);

  int64_t sleepUs = rtcNextReportUs - nowUs();
  if (sleepUs < 1000000LL) sleepUs = 1000000LL;
  esp_sleep_enable_timer_wakeup((uint64_t)sleepUs);

  Serial.printf("Deep Sleep fuer %lu s (Regen-Pegel %d, offen %lu Kippungen)\n",
                (unsigned long)(sleepUs / 1000000LL), rtcRainLevel, (unsigned long)rtcTipsPending);
  Serial.flush();
  esp_deep_sleep_start();
}

// --------------------------------------------------------------------------
// Messen: BME280 im Forced Mode (eine Messung, danach schlaeft der Sensor),
// Anemometer und Batterie als kurze Mittelwerte.
// --------------------------------------------------------------------------
static void measure() {
  if (g_bmeOk) {
    bme.takeForcedMeasurement();
    g_temp  = bme.readTemperature();
    g_press = bme.readPressure() / 100.0F;
    g_alt   = bme.readAltitude(SEALEVELPRESSURE_HPA);
    g_hum   = bme.readHumidity();
  }

  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(ANEMOMETER);
  // ESP32-H2: 12-bit ADC (0-4095), 3.3V Referenz, Vollbereich = 25 m/s
  g_wind = (sum / 8.0f / 4095.0f) * 3.3f * 25.0f;

#if BATTERY_ENABLED
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(BATTERY_PIN);
  g_battery = mv / 8.0f / 1000.0f * BATTERY_DIVIDER;
#endif
}

// --------------------------------------------------------------------------
// Nutzlast: "rain" ist die Menge SEIT DER LETZTEN quittierten Meldung (mm),
// "tips" die zugehoerige Anzahl Kippungen. Fehlende Werte werden weggelassen.
// --------------------------------------------------------------------------
static void buildJson(char *buf, size_t len, uint32_t tips) {
  int n = snprintf(buf, len, "{\"src\":\"%s\"", g_hwId);
  if (g_bmeOk) {
    n += snprintf(buf + n, len - n, ",\"temp\":%.2f,\"pressure\":%.2f,\"altitude\":%.2f,\"humidity\":%.2f",
                  (double)g_temp, (double)g_press, (double)g_alt, (double)g_hum);
  }
  n += snprintf(buf + n, len - n, ",\"wind\":%.2f,\"rain\":%.2f,\"tips\":%lu",
                (double)g_wind, tips * RAIN_MM_PER_TIP, (unsigned long)tips);
  if (!isnan(g_battery)) n += snprintf(buf + n, len - n, ",\"battery\":%.2f", (double)g_battery);
  snprintf(buf + n, len - n, "}");
}

// --------------------------------------------------------------------------
// Als Sleepy End Device konfigurieren (VOR OThread.start()).
// --------------------------------------------------------------------------
static bool configureSleepyEndDevice() {
  otInstance *inst = OThread.getInstance();
  if (!inst) return false;

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
  }
  return ok;
}

static void setPollPeriod(uint32_t ms) {
  otInstance *inst = OThread.getInstance();
  if (!inst) return;
  if (esp_openthread_lock_acquire(portMAX_DELAY)) {
    otLinkSetPollPeriod(inst, ms);
    esp_openthread_lock_release();
  }
}

// --------------------------------------------------------------------------
// Thread starten. Ohne Dataset: ein Commissioning-Versuch pro Wachphase.
// --------------------------------------------------------------------------
static bool joinNetwork() {
  OThread.begin(false);
  OThread.networkInterfaceUp();

  if (!OThread.hasActiveDataset()) {
#if CONFIG_OPENTHREAD_JOINER
    Serial.println("Kein Dataset im NVS - Commissioning (Anmeldefenster am Collector: 'c').");
    otError err = OThread.startJoiner(JOINER_PSKD, JOINER_TIMEOUT_MS);
    if (err != OT_ERROR_NONE) {
      Serial.printf("Commissioning fehlgeschlagen (%d) - naechster Versuch beim naechsten Aufwachen.\n", (int)err);
      return false;
    }
    Serial.println("Commissioning erfolgreich, Dataset erhalten.");
#else
    Serial.println("CONFIG_OPENTHREAD_JOINER ist im Core-Build nicht aktiviert.");
    return false;
#endif
  }

  configureSleepyEndDevice();
  OThread.start();

  uint32_t start = millis();
  while (OThread.otGetDeviceRole() < OT_ROLE_CHILD) {
    if (millis() - start > ATTACH_TIMEOUT_MS) {
      Serial.println("Timeout - keine Thread-Verbindung.");
      return false;
    }
    delay(20);
  }
  Serial.printf("Verbunden als %s nach %lu ms\n", OThread.otGetStringDeviceRole(),
                (unsigned long)(millis() - start));
  return true;
}

// --------------------------------------------------------------------------
// Senden. Quittierte Kippungen werden abgezogen; was waehrenddessen dazukam,
// bleibt fuer die naechste Meldung stehen.
// --------------------------------------------------------------------------
static bool pushMeasurement() {
  uint32_t tips = rtcTipsPending;
  char json[256];
  buildJson(json, sizeof(json), tips);

  IPAddress collector = OThread.getLeaderRloc();
  setPollPeriod(SED_TX_POLL_PERIOD_MS);
  int code = coap.POST(collector, INGEST_PATH, json);
  setPollPeriod(SED_POLL_PERIOD_MS);

  if (code > 0) {
    noInterrupts();   // rainSample may count concurrently
    rtcTipsPending -= tips;
    interrupts();
    rtcSent++;
    Serial.printf("TX   -> %s  code=%d  %s\n", collector.toString().c_str(), code, json);
    return true;
  }
  rtcFailed++;
  Serial.printf("FAIL -> %s  code=%d (gesendet=%lu fehlgeschlagen=%lu)\n", collector.toString().c_str(),
                code, (unsigned long)rtcSent, (unsigned long)rtcFailed);
  return false;
}

// --------------------------------------------------------------------------
void setup() {
  pinMode(RAIN_PIN, INPUT);   // gauge drives both levels; no pull resistor needed
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  // Rain tip while sleeping: count it and go back to sleep without radio
  if (cause == ESP_SLEEP_WAKEUP_EXT1) {
    countRainLevel(stableRainLevel());
    if (nowUs() < rtcNextReportUs) goToSleep();
  }

  bool resetStart = (cause != ESP_SLEEP_WAKEUP_EXT1 && cause != ESP_SLEEP_WAKEUP_TIMER);
  Serial.begin(115200);
  if (resetStart) delay(1500);  // give the USB console time to attach
  readHardwareId();
  Serial.printf("=== ESP-BME %s: Deep-Sleep-Zyklus (Wecker: %d) ===\n", g_hwId, (int)cause);

  // WS2812 keeps its last colour; a pin level does not switch it off, a black frame does
  rgbLedWrite(LED_PIN, 0, 0, 0);

  if (rtcRainLevel < 0) rtcRainLevel = stableRainLevel();
  const esp_timer_create_args_t rainTimerArgs = { .callback = rainSample, .name = "rain" };
  esp_timer_create(&rainTimerArgs, &rainTimer);
  esp_timer_start_periodic(rainTimer, 20 * 1000);

  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);
  g_bmeOk = bme.begin(BME_ADDR, &I2CBME);
  if (g_bmeOk) {
    bme.setSampling(Adafruit_BME280::MODE_FORCED, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::FILTER_OFF);
  } else {
    Serial.println("BME280 nicht gefunden - sende ohne T/p/rF.");
  }

  measure();
  rtcNextReportUs = nowUs() + (int64_t)reportIntervalS() * 1000000LL;

  coap.setTimeout(COAP_TIMEOUT_MS);
  if (joinNetwork()) pushMeasurement();

  if (resetStart) {
    // Stay reachable for a while after a reset (console, flashing)
    g_stayAwake = true;
    g_awakeUntilMs = millis() + STARTUP_AWAKE_S * 1000UL;
    Serial.printf("Bleibe %d s wach, dann Deep Sleep. Regenmesser an GPIO%d, Pegel %d.\n",
                  STARTUP_AWAKE_S, RAIN_PIN, rtcRainLevel);
    return;
  }
  Serial.printf("Wachzeit %lu ms\n", (unsigned long)millis());
  goToSleep();
}

// --------------------------------------------------------------------------
// Wird nur im Wachfenster nach einem RESET erreicht.
// --------------------------------------------------------------------------
void loop() {
  if (g_stayAwake && (int32_t)(millis() - g_awakeUntilMs) >= 0) goToSleep();
  delay(50);
}
