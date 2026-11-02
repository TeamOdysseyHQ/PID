#pragma once

#include <cstddef>
#include <cstdint>

// Separate from PIDConfig at bytes 0..75. Sensor-specific data stays in EEPROM.
struct ImuCalibration {
  static constexpr int EEPROM_ADDRESS = 512;
  uint8_t bytes[32] = {};

  uint8_t* offsets() { return bytes + 8; }
  const uint8_t* offsets() const { return bytes + 8; }

  uint16_t checksum() const {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < 30; ++i) {
      crc ^= static_cast<uint16_t>(bytes[i]) << 8;
      for (unsigned bit = 0; bit < 8; ++bit)
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
  }

  void seal() {
    bytes[0] = 'B'; bytes[1] = 'N'; bytes[2] = 'O'; bytes[3] = '1';
    bytes[4] = 1;     // Format version.
    bytes[5] = 0x80;  // UNIT_SEL.
    bytes[6] = 0x24;  // AXIS_MAP_CONFIG.
    bytes[7] = 0;     // AXIS_MAP_SIGN.
    const uint16_t crc = checksum();
    bytes[30] = crc & 0xFF;
    bytes[31] = crc >> 8;
  }

  bool valid() const {
    return bytes[0] == 'B' && bytes[1] == 'N' && bytes[2] == 'O' &&
           bytes[3] == '1' && bytes[4] == 1 && bytes[5] == 0x80 &&
           bytes[6] == 0x24 && bytes[7] == 0 &&
           checksum() == (static_cast<uint16_t>(bytes[31]) << 8 | bytes[30]);
  }
};

static_assert(sizeof(ImuCalibration) == 32, "EEPROM format must stay 32 bytes");
