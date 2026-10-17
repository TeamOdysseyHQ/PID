#pragma once

#include <cmath>
#include <cstdint>
#include <i2c_driver.h>

// BNO055 page 0 register layout and scales, Bosch datasheet revision 1.8.
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
// The bus and buffers belong to this driver until each async transfer finishes.
class AsyncBno055 {
public:
  struct Sample {
    double orientation[4];  // w, x, y, z in the sensor's native world frame
    double angularVelocity[3];  // rad/s
    double acceleration[3];  // m/s^2, including gravity
    uint32_t startedMs;
    bool calibrated;
  };

  explicit AsyncBno055(I2CMaster& bus) : bus_(bus) {}

  void begin(uint32_t now) {
    bus_.begin(400000);
    phase_ = Phase::Probe;
    transfer_ = Transfer::Idle;
    readyAt_ = now + 850;
  }

  // Returns once per complete, valid sample. Never waits for I2C or a delay.
  bool poll(uint32_t now) {
    if (transfer_ != Transfer::Idle) {
      if (!bus_.finished()) {
        if (now - transferStarted_ >= 10) fail(now);
        return false;
      }
      if (bus_.has_error() || bus_.get_bytes_transferred() != expectedBytes_) {
        fail(now);
        return false;
      }
      if (transfer_ == Transfer::Selecting) {
        transfer_ = Transfer::Reading;
        transferStarted_ = now;
        if (phase_ == Phase::Decode) sample_.startedMs = now;
        expectedBytes_ = readLength_;
        bus_.read_async(ADDRESS, rx_, readLength_, true);
        return false;
      }
      transfer_ = Transfer::Idle;
      readyAt_ = now + settleMs_;
    }

    if (static_cast<int32_t>(now - readyAt_) < 0) return false;

    switch (phase_) {
      case Phase::Retry:
        begin(now);
        break;
      case Phase::Probe:
        read(0x00, 1, Phase::CheckId, now);
        break;
      case Phase::CheckId:
        if (rx_[0] != 0xA0) { fail(now); break; }
        write(0x3D, 0x00, Phase::Reset, 25, now);
        break;
      case Phase::Reset:
        write(0x3F, 0x20, Phase::ProbeAfterReset, 850, now);
        break;
      case Phase::ProbeAfterReset:
        read(0x00, 1, Phase::CheckResetId, now);
        break;
      case Phase::CheckResetId:
        if (rx_[0] != 0xA0) { fail(now); break; }
        write(0x07, 0x00, Phase::Power, 0, now);
        break;
      case Phase::Power:
        write(0x3E, 0x00, Phase::Units, 10, now);
        break;
      case Phase::Units:
        // Preserve reset format: Android orientation, m/s^2, degrees/s.
        write(0x3B, 0x80, Phase::Oscillator, 0, now);
        break;
      case Phase::Oscillator:
        write(0x3F, 0x00, Phase::Fusion, 10, now);
        break;
      case Phase::Fusion:
        write(0x3D, 0x0C, Phase::Sample, 25, now);
        break;
      case Phase::Sample:
        nextSampleAt_ = now + 33;
        // One burst covers accel, gyro, quaternion, calibration and health.
        read(0x08, sizeof(rx_), Phase::Decode, now);
        break;
      case Phase::Decode:
        phase_ = Phase::Sample;
        readyAt_ = nextSampleAt_;
        // Reject sensor resets, changed units, and fusion/system errors.
        if ((rx_[0x3D - 0x08] & 0x0F) != 0x0C || rx_[0x3B - 0x08] != 0x80 ||
            rx_[0x39 - 0x08] != 5 || rx_[0x3A - 0x08] != 0) {
          fail(now);
          break;
        }
        return decode();
    }
    return false;
  }

  const Sample& sample() const { return sample_; }

private:
  enum class Phase {
    Probe, CheckId, Reset, ProbeAfterReset, CheckResetId, Power, Units,
    Oscillator, Fusion, Sample, Decode, Retry
  };
  enum class Transfer { Idle, Selecting, Reading, Writing };
  static constexpr uint8_t ADDRESS = 0x28;
  I2CMaster& bus_;
  Phase phase_ = Phase::Retry;
  Transfer transfer_ = Transfer::Idle;
  uint32_t readyAt_ = 0;
  uint32_t transferStarted_ = 0;
  uint32_t settleMs_ = 0;
  uint32_t nextSampleAt_ = 0;
  size_t expectedBytes_ = 0;
  size_t readLength_ = 0;
  uint8_t tx_[2] = {};
  uint8_t rx_[0x3E - 0x08] = {};
  Sample sample_ = {};

  void fail(uint32_t now) {
    // v1.1.0 can reject the first restart as master_not_ready after a timeout.
    // That clears its software state; the following retry can recover.
    bus_.end();
    transfer_ = Transfer::Idle;
    phase_ = Phase::Retry;
    readyAt_ = now + 1000;
  }

  void read(uint8_t reg, size_t length, Phase next, uint32_t now) {
    tx_[0] = reg;
    readLength_ = length;
    expectedBytes_ = 1;
    settleMs_ = 0;
    phase_ = next;
    transfer_ = Transfer::Selecting;
    transferStarted_ = now;
    bus_.write_async(ADDRESS, tx_, 1, false);
  }

  void write(uint8_t reg, uint8_t value, Phase next, uint32_t settle, uint32_t now) {
    tx_[0] = reg;
    tx_[1] = value;
    expectedBytes_ = sizeof(tx_);
    settleMs_ = settle;
    phase_ = next;
    transfer_ = Transfer::Writing;
    transferStarted_ = now;
    bus_.write_async(ADDRESS, tx_, sizeof(tx_), true);
  }

  int16_t value(size_t offset) const {
    return static_cast<int16_t>(static_cast<uint16_t>(rx_[offset]) |
                                (static_cast<uint16_t>(rx_[offset + 1]) << 8));
  }

  bool decode() {
    double normSquared = 0;
    for (size_t i = 0; i < 4; ++i) {
      sample_.orientation[i] = value(0x20 - 0x08 + 2 * i) / 16384.0;
      normSquared += sample_.orientation[i] * sample_.orientation[i];
    }
    if (normSquared < 0.9 || normSquared > 1.1) return false;
    const double norm = std::sqrt(normSquared);
    for (double& component : sample_.orientation) component /= norm;
    for (size_t i = 0; i < 3; ++i) {
      sample_.acceleration[i] = value(2 * i) / 100.0;
      sample_.angularVelocity[i] = value(0x14 - 0x08 + 2 * i) *
                                  (3.14159265358979323846 / (180.0 * 16.0));
    }
    sample_.calibrated = rx_[0x35 - 0x08] == 0xFF;
    return true;
  }
};
