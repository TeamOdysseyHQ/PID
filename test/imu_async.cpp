#include "AsyncBno055.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>

// Use the real I2CMaster interface. Only the hardware transport is simulated.
I2CDriver::I2CDriver() : pad_control_config(0) {}

class FakeBus : public I2CMaster {
public:
  bool pending = false;
  bool reading = false;
  bool stalled = false;
  bool absent = false;
  bool shortRead = false;
  bool stopped = false;
  bool busyAfterReset = false;
  size_t transfers = 0;
  size_t completedBytes = 0;
  uint8_t* buffer = nullptr;
  size_t length = 0;
  uint8_t reg = 0;
  uint8_t registers[256] = {};

  FakeBus() {
    registers[0] = 0xA0;
    registers[0x35] = 0xFF;
    registers[0x39] = 5;
    put(0x20, 16384);  // Identity quaternion.
    put(0x0C, 981);  // Stationary gravity.
    put(0x14, -1440);  // -90 degrees/second.
  }

  void put(uint8_t address, int16_t value) {
    registers[address] = static_cast<uint16_t>(value) & 0xFF;
    registers[address + 1] = static_cast<uint16_t>(value) >> 8;
  }

  void begin(uint32_t frequency) override {
    assert(frequency == 400000);
    pending = false;
    stopped = false;
  }
  void end() override {
    // v1.1.0 resets hardware but retains the private transfer state.
    busyAfterReset = busyAfterReset || pending;
    pending = false;
    stopped = true;
  }
  bool finished() override { return !pending; }
  size_t get_bytes_transferred() override { return completedBytes; }

  void write_async(uint8_t address, uint8_t* data, size_t size, bool stop) override {
    assert(address == 0x28 && !pending && !stopped);
    assert((size == 1 && !stop) || (size == 2 && stop));
    start(data, size, false);
  }
  void read_async(uint8_t address, uint8_t* data, size_t size, bool stop) override {
    assert(address == 0x28 && !pending && !stopped && stop);
    start(data, size, true);
  }

  void start(uint8_t* data, size_t size, bool isRead) {
    if (busyAfterReset) {
      busyAfterReset = false;
      _error = I2CError::master_not_ready;
      completedBytes = 0;
      return;
    }
    _error = I2CError::ok;
    buffer = data;
    length = size;
    reading = isRead;
    pending = true;
    completedBytes = 0;
    ++transfers;
  }

  void complete() {
    if (!pending || stalled) return;
    pending = false;
    if (absent) {
      _error = I2CError::address_nak;
      return;
    }
    completedBytes = reading && shortRead ? length - 1 : length;
    if (reading) {
      std::memcpy(buffer, registers + reg, completedBytes);
    } else {
      reg = buffer[0];
      if (length == 2) {
        registers[reg] = buffer[1];
        if (reg == 0x3D) registers[reg] |= 0x10;  // Reserved mode bit.
      }
    }
  }
};

struct Rig {
  FakeBus bus;
  AsyncBno055 imu{bus};
  uint32_t now = 0;
  size_t motorTicks = 0;
  size_t samples = 0;

  explicit Rig(uint32_t start = 0) : now(start) { imu.begin(now); }
  bool tick() {
    ++now;
    ++motorTicks;  // A motor-loop iteration occurs even with unfinished I2C.
    bus.complete();
    bool ready = imu.poll(now);
    samples += ready;
    return ready;
  }
  void run(size_t ticks) { while (ticks--) tick(); }
  void nextSample() {
    for (size_t i = 0; i < 6000; ++i) if (tick()) return;
    assert(false && "No sample after recovery deadline");
  }
};

int main() {
  {
    Rig rig;
    rig.run(849);
    assert(rig.bus.transfers == 0);  // Startup delay never waits in poll().
    rig.nextSample();
    auto sample = rig.imu.sample();
    assert(sample.calibrated);
    assert(std::abs(sample.orientation[0] - 1) < 1e-12);
    assert(std::abs(sample.acceleration[2] - 9.81) < 1e-12);
    assert(std::abs(sample.angularVelocity[0] + 3.141592653589793 / 2) < 1e-12);
    auto before = rig.samples;
    rig.run(330);
    assert(rig.samples - before == 10);  // 33 ms sampling without catch-up bursts.
    rig.bus.registers[0x35] = 0;
    rig.nextSample();
    assert(!rig.imu.sample().calibrated);
  }
  {
    Rig rig;
    rig.bus.stalled = true;
    rig.run(1000);
    assert(rig.bus.stopped && rig.samples == 0 && rig.motorTicks == 1000);
    assert(rig.bus.transfers == 1);  // Hung transfer is aborted and backed off.
    rig.bus.stalled = false;
    rig.nextSample();
    assert(rig.samples == 1);
  }
  {
    Rig rig;
    rig.bus.absent = true;
    rig.run(1000);
    assert(rig.samples == 0 && rig.bus.stopped);
    rig.bus.absent = false;
    rig.nextSample();  // A sensor connected later recovers without a reboot.
    auto before = rig.samples;
    rig.bus.shortRead = true;
    rig.run(100);
    assert(rig.samples == before && rig.bus.stopped);
    rig.bus.shortRead = false;
    rig.nextSample();
  }
  {
    Rig rig;
    rig.nextSample();
    auto before = rig.samples;
    rig.bus.put(0x20, 0);
    rig.run(100);
    assert(rig.samples == before);  // Zero quaternion is never published.
    rig.bus.put(0x20, 32767);
    rig.run(100);
    assert(rig.samples == before);  // Corrupt norm is never normalized to validity.
    rig.bus.put(0x20, 16384);
    rig.bus.registers[0x3A] = 1;
    rig.run(100);
    assert(rig.samples == before && rig.bus.stopped);
  }
  {
    Rig rig;
    rig.run(850);  // The chip-ID register selector is in flight.
    rig.now += 100;  // Simulate the existing agent-ping call between phases.
    rig.bus.complete();
    assert(!rig.imu.poll(rig.now));  // The read is launched now.
    rig.bus.stalled = true;
    rig.run(8);
    assert(!rig.bus.stopped);  // The read has its own 10 ms budget.
    rig.bus.stalled = false;
    rig.nextSample();
  }
  {
    Rig rig(std::numeric_limits<uint32_t>::max() - 400);
    rig.nextSample();  // Startup deadline crosses millis() rollover.
  }
  {
    Rig rig(std::numeric_limits<uint32_t>::max() - 3000);
    rig.nextSample();
    auto before = rig.samples;
    rig.run(3300);  // Streaming crosses rollover without restarting the driver.
    assert(rig.samples - before == 100);
  }
  std::cout << "PASS: sampling, units, calibration, stalled bus, missing sensor, "
               "short reads, recovery, invalid quaternion, sensor error, clock wrap\n";
}
