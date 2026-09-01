# ESP-BME — Wetterstation über Thread + CoAP

Kompakte Wetterstation auf Basis eines **ESP32-H2** (ESP32-H2FH4S, 96 MHz, 4 MB Flash,
802.15.4/Thread + BLE 5), die Umgebungsdaten über ein Thread-Netzwerk per **CoAP**
publiziert. Der Thread Border Router (OTBR) im lokalen Netz empfängt die Daten und
leitet sie weiter (z. B. in eine Datenbank).

## Sensoren / Messgrößen

| Messgröße | Sensor | CoAP-Ressource |
|-----------|--------|----------------|
| Temperatur (°C) | BME280 (I²C) | `/weather/temp` |
| Luftfeuchte (%) | BME280 (I²C) | `/weather/humidity` |
| Luftdruck (hPa) | BME280 (I²C) | `/weather/pressure` |
| Windgeschwindigkeit (m/s) | Anemometer (ADC1_CH0) | `/weather/all` (JSON) |
| Regenmenge (mm, kumuliert) | Regenmesser (Interrupt) | `/weather/all` (JSON) |

Ein kombinierter JSON-Endpoint `/weather/all` liefert alle fünf Messgrößen in einem
GET (idealer Endpoint für einen DB-Schreiber ert über dem OTBR).

## Pinbelegung

| Funktion | GPIO |
|----------|------|
| I²C SDA (BME280) | 5 |
| I²C SCL (BME280) | 4 |
| Anemometer (Analog, ADC1_CH0) | 1 |
| Regenmesser (Tipping Bucket, Interrupt) | 12 |

## Verdrahtung

- BME280 VCC → 3.3V, GND → GND, SDA → GPIO5, SCL → GPIO4 (I²C-Adresse `0x77`)
- Anemometer Signal → GPIO1 (ADC1_CH0, bei Bedarf mit Pull-Down)
- Regenmesser → GPIO12 (Interrupt, FALLING)

## Kommunikationsarchitektur

Dieses Projekt ist ein **reiner Arduino-IDE-Sketch** (`ESP-BME.ino`) mit dem
offiziellen OpenThread-Arduino-Paket von Espressif:

- **Thread** — der Node tritt einem Thread-Netzwerk bei (typischerweise dem vom
  Thread Border Router geformten Netz) und bekommt eine IPv6-Adresse (Thread-EID).
- **CoAP (Server)** — der Node stellt die Messwerte als CoAP-Ressourcen auf
  `udp/5683` bereit. Das Backend (erreichbar über den OTBR) liest sie per
  `coap-client` bzw. einem beliebigen CoAP-Poll.

Die Übertragung erfolgt ausschließlich innerhalb des Thread-Mesh; der OTBR ist nur
eine Brücke ins IP-Netz — er verarbeitet keine Daten, sondern macht die CoAP-Ressource
des Nodes von außen erreichbar.

### Netzwerk-Topologie

Der **Thread Border Router formt das Thread-Netz**. Der Node **tritt diesem Netz bei**.
Das geschieht hier über ein **Operational Dataset** (Network Name, PAN ID, Channel,
Extended PAN ID, Network Key) — die Werte des OTBR-Netzes müssen in `ESP-BME.ino`
unter `THREAD_*` eingetragen werden. Alternativ steht das Kommissionieren per
**Joiner/PSKd** (OpenThread `startJoiner`) zur Verfügung.

> **Wichtig:** Ein Thread-Node ist (bewusst) **nicht von außen über Wi-Fi/IP
> erreichbar** — nur über einen Thread-Server bzw. den Border Router. Für reine
> „fire-and-forget“-Datenübertragung über das lokale WLAN wäre ein Wi-Fi-Gerät
> (z. B. ESP32) die einfachere Wahl.

## Benötigte Umgebung

- **Arduino IDE** (oder Arduino CLI) mit Board-Paket **esp32** (arduino-esp32 **v3.x**)
  und **ESP32-H2** als Board.
- Der Arduino-Core muss mit **OpenThread-Unterstützung** gebaut sein (Standard bei
  ESP32-H2/C6/C5 in v3.x; ggf. per Boards-Manager-Option / sdkconfig True).
- **Thread Border Router (OTBR)** im lokalen Netz (z. B. Home Assistant mit dem
  Thread-Border-Router-Add-on) — er formt das Thread-Netz und stellt den Zugang ins
  lokale Netz her.
- Arduino-Bibliotheken: **Adafruit BME280 Library** und **Adafruit Unified Sensor**.

## Build & Flash (Arduino IDE)

1. Board **ESP32-H2** (z. B. `ESP32-H2 Dev Module`) und passenden Port wählen.
2. `ESP-BME.ino` öffnen (alle Dateien liegen im Projektordner `ESP-BME/`).
3. Unter `Thread-Netzkonfiguration` die `THREAD_*`-Werte deines Thread-Netzes eintragen
   (vom OTBR, z. B. Home Assistant → Settings → Thread).
4. Verifizieren und auf den ESP32-H2 flashen.

### Kommandozeile (Arduino CLI, optional)

```bash
arduino-cli compile --fqbn esp32:esp32:esp32h2 ESP-BME
arduino-cli upload  -p <COM-Port> --fqbn esp32:esp32:esp32h2 ESP-BME
```

## Ablauf in der Praxis

1. Firmware flashen, Seriell-Monitor (115200 baud) öffnen.
2. Seriell-Log zeigt den Thread-Beitritt und die Netzwerkinformationen.
3. Den Thread-EID/RLOC des Nodes notieren.
4. Vom Backend (über den OTBR erreichbar) die CoAP-Ressourcen pollen, z. B. mit
   `coap-client` aus dem Thread-Server/Gateway-Netzwerk:
   ```bash
   coap-client -m get coap://[<thread-eid>]/weather/all
   ```

## Energiesparen

- Zwischen den Abtast-/Report-Zyklen bleibt die CPU im Leerlauf (Standard-Light-Sleep
  von arduino-esp32 unter Thread).
- Für Batteriebetrieb `REPORT_INTERVAL_MS` und `SAMPLING_COUNT`/`SAMPLING_DELAY`
  erhöhen (weniger Radio-Aktivität).
- Hinweis: Ein **Sleepy End Device (SED)**-Betrieb mit minimalem Stromverbrauch
  erfordert die OpenThread-Konfiguration des Cores (Poll-Period u. Ä.) und ist hier
  bewusst einfach gehalten.

## Geschichte

Der Code wurde von einer ESP-NOW-Lösung („fire-and-forget“) über **Matter over
Thread** (eigener Zweig) schließlich zur einheitlichen, reinen **Thread + CoAP**-Variante
portiert — Matter (mit Custom Cluster + ESP-IDF-Build) war für einen reinen Sensorknoten,
der nur Daten senden soll, überdimensioniert.
