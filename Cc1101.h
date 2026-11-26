// Minimal CC1101 driver: 433.92 MHz OOK receiver in asynchronous serial mode.
// The demodulated signal comes out on GDO0; the sketch times its edges.
#pragma once
#include <Arduino.h>
#include <SPI.h>

class Cc1101 {
public:
  Cc1101(int sck, int miso, int mosi, int cs) : sck_(sck), miso_(miso), mosi_(mosi), cs_(cs) {}

  // Resets the chip and configures it; false if it does not answer on SPI.
  bool begin() {
    pinMode(cs_, OUTPUT);
    digitalWrite(cs_, HIGH);
    SPI.begin(sck_, miso_, mosi_, -1);
    // manual reset sequence (datasheet 19.1.2)
    digitalWrite(cs_, LOW);  delayMicroseconds(10);
    digitalWrite(cs_, HIGH); delayMicroseconds(40);
    strobe(SRES);
    delay(1);
    partnum_ = status(0x30);
    version_ = status(0x31);
    if (partnum_ != 0x00 || version_ == 0x00 || version_ == 0xFF) return false;

    static const uint8_t cfg[][2] = {
      {0x00, 0x2E},  // IOCFG2: high impedance
      {0x02, 0x0D},  // IOCFG0: asynchronous serial data out
      {0x03, 0x47},  // FIFOTHR
      {0x07, 0x00},  // PKTCTRL1
      {0x08, 0x32},  // PKTCTRL0: asynchronous serial mode, infinite length
      {0x0B, 0x06},  // FSCTRL1: IF
      {0x0C, 0x00},  // FSCTRL0
      {0x0D, 0x10}, {0x0E, 0xB0}, {0x0F, 0x71},  // FREQ: 433.92 MHz (26 MHz crystal)
      {0x10, 0x77},  // MDMCFG4: RX bandwidth 232 kHz
      {0x11, 0x93},  // MDMCFG3
      {0x12, 0x30},  // MDMCFG2: ASK/OOK, no sync word
      {0x13, 0x22},  // MDMCFG1
      {0x14, 0xF8},  // MDMCFG0
      {0x18, 0x18},  // MCSM0: calibrate when leaving IDLE
      {0x19, 0x16},  // FOCCFG
      {0x1A, 0x6C},  // BSCFG
      {0x1B, 0x43},  // AGCCTRL2
      {0x1C, 0x00},  // AGCCTRL1
      {0x1D, 0x91},  // AGCCTRL0: OOK decision boundary 8 dB
      {0x21, 0x56},  // FREND1
      {0x22, 0x11},  // FREND0
      {0x23, 0xE9}, {0x24, 0x2A}, {0x25, 0x00}, {0x26, 0x1F},  // FSCAL
      {0x2C, 0x81}, {0x2D, 0x35}, {0x2E, 0x09},  // TEST2..0
    };
    for (auto &r : cfg) write(r[0], r[1]);
    return true;
  }

  void receive() {
    strobe(SIDLE);
    strobe(SRX);
  }

  // Power down (~200 nA) until the next begin()
  void sleep() {
    strobe(SIDLE);
    strobe(SPWD);
    SPI.end();
  }

  uint8_t partnum() const { return partnum_; }
  uint8_t version() const { return version_; }

private:
  enum { SRES = 0x30, SRX = 0x34, SIDLE = 0x36, SPWD = 0x39 };

  void select() {
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_, LOW);
    // the chip pulls MISO low once its crystal runs
    for (uint32_t t = micros(); digitalRead(miso_) && micros() - t < 2000;) {}
  }
  void deselect() {
    digitalWrite(cs_, HIGH);
    SPI.endTransaction();
  }
  void strobe(uint8_t cmd) { select(); SPI.transfer(cmd); deselect(); }
  void write(uint8_t addr, uint8_t value) { select(); SPI.transfer(addr); SPI.transfer(value); deselect(); }
  uint8_t status(uint8_t addr) {
    select();
    SPI.transfer(addr | 0xC0);
    uint8_t v = SPI.transfer(0);
    deselect();
    return v;
  }

  int sck_, miso_, mosi_, cs_;
  uint8_t partnum_ = 0xFF, version_ = 0;
};
