# Schaltplan — ESP-BME Wetterstation (ESP32-H2)

Verdrahtung der Sensoren am **ESP32-H2-Modul (Supermini-Bauform)**, passend zum
Sketch `ESP-BME.ino` (Thread + CoAP, Deep Sleep).

> **Alles, was an einem GPIO hängt, darf dort höchstens 3,3 V liefern.**
> Der ESP32-H2 ist nicht 5-V-tolerant. Sensoren und das RS485-Modul deshalb an
> **3V3** versorgen – nicht an 5V. Einzige Ausnahme ist die Windfahne mit eigenem
> 12-V-Netzteil; sie hängt nur am RS485-Bus, nicht an einem GPIO.

## Board-Pinbelegung

Draufsicht, USB-C oben. Gezählt wird je Leiste von oben nach unten.

```text
                    ┌───[USB-C]───┐
          GP24 [1]  │ TX       5V │ [1]  5V
          GP23 [2]  │ RX      GND │ [2]  GND        <- GND-Schiene
          GP0  [3]  │         3V3 │ [3]  3V3        <- 3V3-Schiene
Anemometer GP1 [4]  │        GP14 │ [4]  GP14
          GP2  [5]  │        GP13 │ [5]  GP13
          GP3  [6]  │        GP12 │ [6]  GP12       <- Regenmesser
BME280 SCL GP4 [7]  │        GP11 │ [7]  GP11 (RX)  <- RS485-Modul
BME280 SDA GP5 [8]  │        GP10 │ [8]  GP10 (TX)  -> RS485-Modul
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
| Versorgung 3,3 V | 3V3 | rechts 3 | BME280 VCC, RS485-Modul VCC, Regenmesser (Seite A) |
| Masse | GND | rechts 2 | alle Sensoren, RS485-Modul, **Minus des 12-V-Netzteils** |
| I²C SCL | GP4 | links 7 | BME280 SCL |
| I²C SDA | GP5 | links 8 | BME280 SDA |
| Anemometer (analog) | GP1 (ADC1_CH0) | links 4 | Signal, max. 3,3 V |
| Regenmesser | GP12 | rechts 6 | Wippe (weckt aus dem Deep Sleep) |
| Windfahne UART1 TX | GP10 | rechts 8 | RS485-Modul, Eingang (siehe unten) |
| Windfahne UART1 RX | GP11 | rechts 7 | RS485-Modul, Ausgang (siehe unten) |

## Sensoren

### BME280 (Temperatur, Luftdruck, Luftfeuchte)

- VCC → 3V3, GND → GND, SCL → GP4, SDA → GP5.
- Adresse **0x77** (SDO auf High) oder **0x76** (SDO auf Low); die Firmware probiert beide.
- Die meisten Breakouts haben 4,7-kΩ-Pull-ups onboard, sonst je 4,7 kΩ von SDA und SCL nach 3V3.

### Anemometer (Windgeschwindigkeit)

- Signal → GP1, Masse → GND. Der Sketch rechnet `0–3,3 V = 0–25 m/s`.
- Das Signal darf **3,3 V nicht überschreiten**. Bei höherer Ausgangsspannung
  einen Spannungsteiler vorschalten:

```text
 Anemometer Signal ── 10 kΩ ──┬── GP1
                              │
                            6,8 kΩ
                              │
                             GND        (5 V → ca. 2,0 V am ADC)
```

### Regenmesser (Wippe)

- Die Wippe schaltet GP12 zwischen **3V3 (Seite A)** und **GND (Seite B)** um.
- Jeder stabile Pegelwechsel (≥ 100 ms) zählt als Kippung = 0,45 mm.
- Kein Pull-up nötig; GP12 kann den Chip aus dem Deep Sleep wecken.

### Windfahne (RS485, Modbus RTU)

Windfahne mit RS485-Ausgang (10–30 V, Modbus RTU, Adresse 1, 4800 Baud 8N1).
Der Sketch liest sie bei jedem Aufwachen über UART1 und sendet `winddir`
(Grad, 0 = Nord). Er probiert 4800/9600/2400 Baud und die Funktionen 03/04
und merkt sich, was antwortet. Ein RS485-TTL-Modul mit automatischer
Richtungsumschaltung setzt die Pegel um.

```text
 ESP32-H2                RS485-TTL-Modul               Windfahne
 3V3  (rechts 3) ─────── VCC
 GND  (rechts 2) ─────── GND
 GP10 (rechts 8) ─────── TXD*          A+ ─────────── gelb  (485-A)
 GP11 (rechts 7) ─────── RXD*          B− ─────────── blau  (485-B)
                                       (3. Klemme 地/GND: frei lassen)
 GND ─────────────────────────────────────────┐
                        12-V-Netzteil  − ─────┴────── schwarz
                                       + ──────────── braun
```

\* Die Beschriftung TXD/RXD ist bei diesen Modulen uneinheitlich. Richtig ist
die Belegung, bei der nach einem RESET (Sendeversuch) eine LED des Moduls
blinkt. Antwortet die Windfahne, blinkt kurz danach auch die zweite LED.

- **Modul-VCC an 3V3, nicht an 5V.** Bei 5 V liegt am Ausgang des Moduls 5 V
  an GP11 an, und die 3,3-V-Pegel des H2 werden eventuell nicht erkannt.
- Die Windfahne braucht ein **eigenes 10–30-V-Netzteil**; dessen Minus gehört
  auf die gemeinsame GND-Schiene. Plus (braun) nie an das Modul oder den H2.
- Aderfarben je nach Hersteller: A ist gelb oder grün, B ist blau.
- Ruhepegel bei angeschlossenem Bus (gegen GND gemessen): A+ ≈ 3 V, B− ≈ 1 V.
- Die serielle Konsole zeigt bei jedem Zyklus `Windfahne ...` mit den Registern
  oder, wenn nichts zurückkommt, die empfangenen Rohbytes.

## Stromversorgung

- Das Modul über **USB-C** (5 V) versorgen oder alternativ 5 V an den 5V-Pin
  (rechts 1). Die 3V3-Schiene liefert der Onboard-Regler.
- Nur der 5V-Pin selbst verträgt 5 V; an die GPIOs gehört höchstens 3,3 V.
- Die Windfahne hat ihr eigenes 12-V-Netzteil (gemeinsame Masse, siehe oben).

## Prüfen vor dem Start

1. Mit dem Multimeter an **jedem belegten GPIO gegen GND**: höchstens 3,3 V.
2. 3V3 gegen GND: ca. 3,3 V; 5V gegen GND: ca. 5 V (bei USB-Versorgung).
3. Windfahne: braun gegen schwarz 10–30 V; A+/B− gegen GND im Ruhezustand ca. 3 V / 1 V.
4. Nach RESET bleibt der Knoten 30 s wach – in diesem Fenster flashen und die
   Konsole lesen.
