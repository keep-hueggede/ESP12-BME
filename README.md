# ESP32-H2 BME

Arduino IDE Sketch to read from BME280 sensor and send per ESP-NOW to Receiver.

## Board
- **ESP32-H2** with ESP32-H2FH4S chip (96MHz, 4MB Flash)
- GPIO4 = I2C SDA, GPIO5 = I2C SCL
- ADC1_CH0 = GPIO36 (Anemometer input)

## MAC's
* Receiver: 
  *  0x84, 0x0D, 0x8E, 0xB7, 0xFE, 0x1B
* Sender:
  * 0x84, 0xF3, 0xEB, 0x05, 0x43, 0xE8

## Pinout
| Function | GPIO |
|----------|------|
| I2C SDA  | 4    |
| I2C SCL  | 5    |
| Anemometer | 36 (ADC1_CH0) |

## Wiring
- BME280 VCC → 3.3V
- BME280 GND → GND
- BME280 SDA → GPIO4
- BME280 SCL → GPIO5
- Anemometer signal → GPIO36 (with pull-down if needed)