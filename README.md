# ESP-BME — Weather station über Matter over Thread

Kompakte Wetterstation auf Basis eines **ESP32-H2** (ESP32-H2FH4S, 96 MHz, 4 MB Flash,
802.15.4/Thread + BLE 5), die Umgebungsdaten als **Matter over Thread**-Device
publiziert. Der Fokus liegt auf möglichst sparsamem Betrieb (Thread Sleepy End Device
+ Light Sleep).

## Sensoren / Messgrößen

| Messgröße | Sensor | Matter-Abbildung |
|-----------|--------|------------------|
| Temperatur | BME280 (I²C) | Standard-Endpoint `MatterTemperatureSensor` |
| Luftfeuchte | BME280 (I²C) | Standard-Endpoint `MatterHumiditySensor` |
| Luftdruck | BME280 (I²C) | Standard-Endpoint `MatterPressureSensor` |
| Windgeschwindigkeit | Anemometer (ADC1_CH0) | Vendor Custom Cluster (`0x131BFC01`, Attribut `0x0000`, Wert = m/s · 10) |
| Regenmenge | Regenmesser (Interrupt) | Vendor Custom Cluster (`0x131BFC01`, Attribut `0x0001`, Wert = mm · 10; 1 Kipp = 0,45 mm) |

Matter bietet für **Windgeschwindigkeit** und **Regenmenge** keine standardisierten
Endpoints, daher werden diese beiden Messgrößen über einen herstellerspezifischen
Custom Cluster bereitgestellt (Vendor-ID `0x131B` = Espressif-Test-VID). Temperatur,
Luftfeuchte und Druck verwenden die standardisierten Matter-Sensor-Endpoints und sind
damit direkt in beliebigen Matter-Ökosystemen lesbar.

## Pinbelegung

| Funktion | GPIO |
|----------|------|
| I²C SDA (BME280) | 4 |
| I²C SCL (BME280) | 5 |
| Anemometer (Analog, ADC1_CH0) | 36 |
| Regenmesser (Tipping Bucket, Interrupt) | 12 |

## Verdrahtung

- BME280 VCC → 3.3V, GND → GND, SDA → GPIO4, SCL → GPIO5
- Anemometer Signal → GPIO36 (bei Bedarf mit Pull-Down)
- Regenmesser → GPIO12 (Interrupt, FALLING)

## Kommunikationsarchitektur

Dieses Projekt ist als **ESP-IDF-Projekt mit dem Arduino Core als Komponente**
aufgebaut (das offizielle Espressif-Muster für Matter over Thread auf ESP32-H2,
analog zu `Arduino_ESP_Matter_over_OpenThread`). Grund: Herstellerspezifische Custom
Clusters und der Sleepy-End-Device-/Light-Sleep-Betrieb erfordern Zugriff auf die
ESP-IDF-Konfiguration (sdkconfig) und die esp-matter Low-Level-API. Der nutzbare
Code bleibt dennoch ein gewöhnlicher Arduino-Matter-Sketch (`#include <Matter.h>`).

- **Commissioning** erfolgt über BLE (Manual Pairing Code / QR-Code) in ein
  Thread-Netzwerk.
- Nach dem Commissioning läuft das Gerät als **Thread Sleepy End Device** (MTD) mit
  Light Sleep → sehr geringer Stromverbrauch zwischen den Daten-Polls.

### Benötigte Umgebung für Matter over Thread

- **Thread Border Router** im lokalen Netz (z. B. Home Assistant mit
  Thread-Border-Router-Add-on) und ein **Matter-Controller** (z. B. Home Assistant,
  Apple Home, Google Home), um das Gerät zu commissen und die Messwerte auszulesen.
- Ohne Thread-Netzwerk zeigt das Gerät zunächst nur den Pairing-Code an und wartet
  auf Commissioning (BLE).

## Build & Flash (ESP-IDF)

Voraussetzungen: ESP-IDF (z. B. v5.5+) und eine Internetverbindung zum Laden der
Komponenten (esp_matter, arduino-esp32) über den Komponenten-Registry.

```bash
# Projekt aus dem Repo, Ziel setzen (ESP32-H2), bauen, flashen, Seriell-Monitor
idf.py set-target esp32h2
idf.py build
idf.py -p <COM-Port> flash monitor
```

Beim ersten Build werden die deklarierten Komponenten aus
`main/idf_component.yml` heruntergeladen. Die Versionsangaben dort sollten zur
installierten ESP-IDF passen (siehe Freigabedaten von `arduino-esp32`).

## Commissioning in ein Thread-Netzwerk

1. Firmware flashen und Seriell-Monitor öffnen (115200 baud).
2. Den angezeigten **Manual Pairing Code** bzw. die **QR-Code-URL** notieren.
3. Im Matter-Controller (z. B. Home Assistant → Settings → Devices → Add Device,
   „Matter“) das Gerät hinzufügen und den Code bzw. QR-Code eingeben.
4. Nach erfolgreichem Commissioning verbindet sich das Gerät über Thread und wird
   als Sleepy End Device betrieben.
5. Temperatur, Feuchte und Druck erscheinen als eigene Sensoren; Wind & Regen sind
   über den Custom Cluster auslesbar (z. B. mit dem CHIP-Tool).

## Energiesparen

- Thread-Clients nutzen Timer-basierte Polls; der 802.15.4-Radio ist zwischen den
  Polls abgeschaltet (Sleepy End Device).
- `CONFIG_PM_ENABLE` + `light_sleep_enable` lassen die CPU im Leerlauf schlafen.
- Für Batteriebetrieb können `THREAD_POLL_INTERVAL_MS` und die Abtastrate
  (`SAMPLING_COUNT`/`SAMPLING_DELAY`) erhöht werden.

## Factory Reset / Decommissioning

Eine erneute Inbetriebnahme wird durch ein Decommissioning in der Matter-Umgebung
bzw. das Löschen des NVS erreicht:

```bash
idf.py -p <COM-Port> erase-flash
```

## Geschichte

Der Code wurde von einer ESP-NOW-Lösung („fire-and-forget“) auf **Matter over
Thread** portiert und dabei optimiert (Sleepy End Device, Light Sleep, reduzierte
Log-Platform).
