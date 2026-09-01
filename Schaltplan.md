# Schaltplan — ESP-BME Wetterstation (ESP32-H2)

Verdrahtung der Sensoren an den ESP32-H2 (ESP32-H2FH4S), passend zum Sketch
`ESP-BME.ino` (Thread + CoAP).

## Pin-Zuordnung

| Funktion        | ESP32-H2 Pin | Sensor/Anschluss                                      |
|-----------------|--------------|-------------------------------------------------------|
| I²C SDA         | GPIO4        | BME280 SDA (Pull-up 4,7k gegen 3V3, falls nicht onboard) |
| I²C SCL         | GPIO5        | BME280 SCL (Pull-up 4,7k gegen 3V3, falls nicht onboard) |
| Anemometer (Analog) | **GPIO1** (ADC1_CH0) | Anemometer Signal (0–3,3V) |
| Regenmesser (Interrupt) | GPIO12      | Regenmesser Kontakt (schließt nach GND, FALLING)     |
| Masse           | GND          | gemeinsame Masse aller Sensoren                       |
| Versorgung 3,3V | 3V3          | BME280 VCC, Anemometer VCC                            |

> **Hinweis zu GPIO1:** ADC1_CH0 ist am ESP32-H2 der **GPIO1** — nicht GPIO36
> (GPIO36 kommt nur am klassischen ESP32 vor). Der Sketch verwendet `ADC1_CH0`
> und löst damit automatisch auf GPIO1 auf.

## Stromversorgung

- ESP32-H2 über USB-C oder 5V/GND versorgt (DevKitM-1: 5V-Pin, alternativ VBAT
  mit externer 3,3V-Quelle).
- **BME280:** 3,3V (VCC → 3V3, GND → GND). Nicht an 5V anschließen.
- **Anemometer:** VCC → 3V3, GND → GND, Signal → GPIO1. Das analoge Ausgangssignal
  muss im Bereich **0–3,3V** liegen (bei 0–2,5V-Modellen direkt anschließbar; bei
  höheren Ausgangsspannungen Spannungsteiler verwenden).
- **Regenmesser (Tipping Bucket):** Reed-Kontakt bzw. Halbleiter-Schalter, ein
  Anschluss auf GPIO12, der andere auf GND. Der Sketch aktiviert den internen
  Pull-up (INPUT_PULLUP); jeder fallende Flanke (Kontakt schießt zu GND) zählt
  einen Tipp (0,45 mm).

## Verdrahtungsplan (ASCII)

```
 ┌─────────────┐        ┌──────────────────────────────────────────────┐
 │  ESP32-H2   │        │            Wetter-Sensoren                   │
 │  (Modul)    │        │                                              │
 │             │  3V3 ──┼─► BME280 VCC           Anemometer VCC ──────► 3V3 rail
 │  GPIO4 ─────┼── SDA──┼──► BME280 SDA (4k7 Pull-up → 3V3, falls nötig)
 │  GPIO5 ─────┼── SCL──┼──► BME280 SCL (4k7 Pull-up → 3V3, falls nötig)
 │             │        │                                              │
 │  GPIO1 ─────┼────────┼──► Anemometer Signal (0–3,3V analog)         │
 │  (ADC1_CH0) │        │                                              │
 │             │        │                                              │
 │  GPIO12 ────┼────────┼──► Regenmesser Kontakt ──► GND              │
 │             │        │   (Reed-Schalter, schließt nach GND)         │
 │  GND    ────┼──► alle GND (BME280, Anemometer, Regenmesser) ────────┼─► GND rail
 └─────────────┘        └──────────────────────────────────────────────┘
```

### Detaillierte Verbindungen

| Von (ESP32-H2) | An                            | Kabel |
|----------------|-------------------------------|-------|
| 3V3            | BME280 VCC                    | rot   |
| GND            | BME280 GND                    | schwarz |
| GND            | Anemometer GND                | schwarz |
| GND            | Regenmesser (ein Anschluss)   | schwarz |
| GPIO4          | BME280 SDA                    | blau  |
| GPIO5          | BME280 SCL                    | gelb  |
| GPIO1 (ADC1_CH0) | Anemometer Signal           | grün  |
| GPIO12         | Regenmesser (anderer Anschluss) | weiß |

## Prüfwerte vor dem Start

1. **BME280:** erwartete I²C-Adresse `0x76` (Schematics rechnen mit 0x76; bei
   Adressierung über Jumper prüfen).
2. **I²C Pull-ups:** Die meisten BME280-Breakouts haben 4,7k schon onboard. Wenn
   du das nackte Sensormodul verwendest: 4,7k von SDA nach 3V3 und von SCL nach
   3V3 löten.
3. **Anemometer:** Das Signal muss beim Betrieb zwischen 0 und 3,3V liegen.
   Bei Modellen mit 0–2,5V/Vollbereich ist das OK; bei 5V-Ausgang den
   Spannungsteiler (z. B. 10k/6,8k) anbringen, sonst ADC-Überlastung.
4. **Regenmesser:** Beim Umlegen des Kippkontakts muss die Flanke auf GPIO12 von
   HIGH auf LOW gehen (interner Pull-up). Bei manchen Wetterstation-Kontakten ist
   zusätzlich ein externer Pull-up (10k gegen 3V3) sinnvoll.

## Spannungsteiler-Anemometer (nur falls nötig)

```
 GPIO1 ─────┬──── 10 kΩ ──── Anemometer Signal
            │
            6,8 kΩ
            │
           GND
 (Verhältnis ≈ 0,405 → 5V max => ~2,0V am ADC)
```

Hinweis: Der Sketch skaliert mit `(v / 4095.0) * 3.3 * 25` und geht von einem
Vollbereich von 25 m/s bei 3,3V aus. Passt das Teilersignal dazu, bleibt die
Windmessung korrekt.