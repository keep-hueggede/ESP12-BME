# Schaltplan — ESP-BME Wetterstation (ESP32-H2)

Verdrahtung der Sensoren am **ESP32-H2-Modul (Supermini-Bauform)**, passend zum
Sketch `ESP-BME.ino` (Thread + CoAP, Deep Sleep).

> **Alles, was an einem GPIO hängt, darf dort höchstens 3,3 V liefern.**
> Der ESP32-H2 ist nicht 5-V-tolerant. Sensoren und das CC1101-Funkmodul deshalb
> an **3V3** versorgen – nicht an 5V.

## Board-Pinbelegung

Draufsicht, USB-C oben. Gezählt wird je Leiste von oben nach unten.

```text
                    ┌───[USB-C]───┐
          GP24 [1]  │ TX       5V │ [1]  5V
          GP23 [2]  │ RX      GND │ [2]  GND        <- GND-Schiene
          GP0  [3]  │         3V3 │ [3]  3V3        <- 3V3-Schiene
Anemometer GP1 [4]  │        GP14 │ [4]  GP14       -> CC1101 CSN
          GP2  [5]  │        GP13 │ [5]  GP13       <- CC1101 MISO
CC1101 GDO0 GP3 [6] │        GP12 │ [6]  GP12       <- Regenmesser
BME280 SCL GP4 [7]  │        GP11 │ [7]  GP11       -> CC1101 MOSI
BME280 SDA GP5 [8]  │        GP10 │ [8]  GP10       -> CC1101 SCK
          GP8  [9]  │         GP9 │ [9]  GP9 (BOOT)
                    └─────────────┘
          Lötpads unten: GP26/GP27 (USB D-/D+), GP22, GP25
```

- **GP8** = Onboard-RGB-LED (WS2812), wird bei jedem Zyklus schwarz geschaltet.
- **GP9** = BOOT-Taster; **GP26/GP27** = USB – nicht belegen.
- **GP23/GP24** = UART0; die Konsole läuft über USB, die Pins bleiben frei.
- **GP2** ist für die Batteriemessung reserviert (`BATTERY_ENABLED`, derzeit aus).

## Pin-Zuordnung

| Funktion | Pin | Leiste/Pos. | Anschluss |
|---|---|---|---|
| Versorgung 3,3 V | 3V3 | rechts 3 | BME280 VCC, CC1101 VCC, Regenmesser (Seite A) |
| Masse | GND | rechts 2 | alle Sensoren, CC1101 |
| I²C SCL | GP4 | links 7 | BME280 SCL |
| I²C SDA | GP5 | links 8 | BME280 SDA |
| Anemometer (analog) | GP1 (ADC1_CH0) | links 4 | Signal, max. 3,3 V |
| Regenmesser | GP12 | rechts 6 | Wippe (weckt aus dem Deep Sleep) |
| CC1101 SCK | GP10 | rechts 8 | SPI-Takt |
| CC1101 MOSI (SI) | GP11 | rechts 7 | SPI-Daten zum Modul |
| CC1101 MISO (SO) | GP13 | rechts 5 | SPI-Daten vom Modul |
| CC1101 CSN | GP14 | rechts 4 | Chip Select |
| CC1101 GDO0 | GP3 | links 6 | demoduliertes 433-MHz-Signal |

## Sensoren

### BME280 (Temperatur, Luftdruck, Luftfeuchte)

- VCC → 3V3, GND → GND, SCL → GP4, SDA → GP5.
- Adresse **0x77** (SDO auf High) oder **0x76** (SDO auf Low); die Firmware probiert beide.
- Die meisten Breakouts haben 4,7-kΩ-Pull-ups onboard, sonst je 4,7 kΩ von SDA und SCL nach 3V3.

### Anemometer (Windgeschwindigkeit)

Funduino-Schalenanemometer: ein kleiner Generator, **keine Versorgung nötig**.
Ausgang 0–2 V, Windgeschwindigkeit = Spannung × 25 m/s.

- Signal → GP1, Masse → GND, **direkt, ohne Spannungsteiler** (2 V < 3,3 V).
- Der Sketch mittelt bei jedem Aufwachen 3 s lang (`wind`) und meldet den
  höchsten Wert in diesem Fenster als Böe (`gust`).
- Bei Windstille sind Werte um 0 normal.
- Wird das Schalenkreuz rückwärts gedreht, kehrt der Generator die Polarität um.
  Schutz für GP1, falls das vorkommen kann:

```text
 Anemometer Signal ── 1 kΩ ──┬── GP1
                             │
                          Schottky-Diode (Kathode an GP1)
                             │
                            GND
```

### Regenmesser (Wippe)

- Die Wippe schaltet GP12 zwischen **3V3 (Seite A)** und **GND (Seite B)** um.
- Jeder stabile Pegelwechsel (≥ 100 ms) zählt als Kippung = 0,45 mm.
- Kein Pull-up nötig; GP12 kann den Chip aus dem Deep Sleep wecken.

### Windrichtung (TFA 30.3251.10 über CC1101, 433 MHz)

Der Funk-Windsender TFA 30.3251.10 (Protokoll LaCrosse TX141W) sendet etwa
alle 30 s Windrichtung und -geschwindigkeit auf 433,92 MHz. Ein CC1101-Modul
empfängt das Signal; der Sketch dekodiert es und sendet `winddir`
(Grad, 0 = Nord). Er koppelt sich nach einem RESET an den ersten gehörten
Sender und ignoriert danach fremde Sender.

```text
 ESP32-H2                 CC1101 (433-MHz-Version)
 3V3  (rechts 3) ──────── VCC
 GND  (rechts 2) ──────── GND
 GP10 (rechts 8) ──────── SCK / SCLK
 GP11 (rechts 7) ──────── MOSI / SI
 GP13 (rechts 5) ──────── MISO / SO
 GP14 (rechts 4) ──────── CSN
 GP3  (links 6)  ──────── GDO0
                          GDO2: frei lassen
                          ANT: 17,3 cm Draht oder 433-MHz-Antenne
```

- **VCC nur an 3V3.** Der CC1101 verträgt keine 5 V.
- Nach den Beschriftungen auf dem Modul verdrahten; die Reihenfolge der Pins
  unterscheidet sich je nach Modulvariante.
- Leitungen kurz halten (≤ 15 cm), die Antenne weg vom H2 und den Kabeln.
- Beim Aufwachen hört der Sketch bis zu 35 s auf ein Paket und legt das
  nächste Aufwachen kurz vor das nächste erwartete Paket.
- Die Konsole zeigt `CC1101 bereit` und je Zyklus `Windsender <id>: Richtung ...`.

## Stromversorgung

- Das Modul über **USB-C** (5 V) versorgen oder alternativ 5 V an den 5V-Pin
  (rechts 1). Die 3V3-Schiene liefert der Onboard-Regler.
- Nur der 5V-Pin selbst verträgt 5 V; an die GPIOs gehört höchstens 3,3 V.

## Prüfen vor dem Start

1. Mit dem Multimeter an **jedem belegten GPIO gegen GND**: höchstens 3,3 V.
2. 3V3 gegen GND: ca. 3,3 V; 5V gegen GND: ca. 5 V (bei USB-Versorgung).
3. Nach RESET bleibt der Knoten 30 s wach – in diesem Fenster flashen und die
   Konsole lesen.
