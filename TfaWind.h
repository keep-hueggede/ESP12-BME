// Decoder for the TFA 30.3251.10 wind sender (= LaCrosse TX141W protocol), 433.92 MHz OOK.
// Pulse-width coding, clock 250 us: 1 = 500 us high + 250 us low, 0 = 250 + 500,
// sync = 750 + 750 (several in a row). One packet = 64 bits + 1 trailing bit:
//   bits  0-3  0000          bits 4-23  random id      bit 24  battery low
//   bits 30-31 type (10=wind) bits 32-43 speed x10      bits 44-55 direction (deg)
//   bits 56-63 CRC-8 (poly 0x31, init 0) over bytes 0-6
// The sender transmits about every 30 s.
#pragma once
#include <stdint.h>

struct TfaWindReading {
  uint32_t id;
  bool     batteryLow;
  uint16_t speedX10;   // raw speed in 0.1 units of the sender
  uint16_t direction;  // degrees
};

class TfaWindDecoder {
public:
  // Feed every level with its duration. Returns true when a valid packet completed.
  bool pulse(bool high, uint32_t us) {
    if (high) { high_ = us; return false; }
    int hc = cls(high_), lc = cls(us);
    high_ = 0;
    if (hc == SYNC && lc == SYNC) {
      bool ok = bits_ ? finish() : false;
      syncs_++;
      return ok;
    }
    if (syncs_ >= 2 && (hc == SHORT || hc == LONG)) {
      if (lc == GAP) { add(hc == LONG); bool ok = finish(); syncs_ = 0; return ok; }
      if (hc != lc && (lc == SHORT || lc == LONG)) { add(hc == LONG); return false; }
    }
    bool ok = finish();
    syncs_ = 0;
    return ok;
  }

  const TfaWindReading &reading() const { return reading_; }

  static uint8_t crc8(const uint8_t *d, int n) {
    uint8_t c = 0;
    for (int i = 0; i < n; i++) {
      c ^= d[i];
      for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x31) : (uint8_t)(c << 1);
    }
    return c;
  }

private:
  enum { NOISE, SHORT, LONG, SYNC, GAP };
  static int cls(uint32_t us) {
    if (us < 120) return NOISE;
    if (us < 380) return SHORT;
    if (us < 620) return LONG;
    if (us < 1000) return SYNC;
    return GAP;
  }

  void add(bool bit) {
    if (bits_ < 64) {
      if (bit) data_[bits_ / 8] |= (uint8_t)(0x80 >> (bits_ % 8));
      else     data_[bits_ / 8] &= (uint8_t)~(0x80 >> (bits_ % 8));
    }
    bits_++;
  }

  bool finish() {
    int n = bits_;
    bits_ = 0;
    if (n < 64 || n > 68) return false;
    const uint8_t *d = data_;
    if ((d[0] & 0xF0) != 0 || (d[3] & 0x03) != 0x02 || crc8(d, 7) != d[7]) return false;
    reading_.id = ((uint32_t)(d[0] & 0x0F) << 16) | ((uint32_t)d[1] << 8) | d[2];
    reading_.batteryLow = d[3] & 0x80;
    reading_.speedX10 = (uint16_t)((d[4] << 4) | (d[5] >> 4));
    reading_.direction = (uint16_t)(((d[5] & 0x0F) << 8) | d[6]);
    return reading_.direction < 360;
  }

  uint32_t high_ = 0;
  int bits_ = 0;
  int syncs_ = 0;
  uint8_t data_[8] = {0};
  TfaWindReading reading_ = {};
};
