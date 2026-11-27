#ifndef I2C_BUS_H
#define I2C_BUS_H

// Shared I2C bus arbitration.
//
// The OLED (Adafruit_SSD1306) and the MPU9250 sit on the same SDA/SCL pins but
// are driven from different FreeRTOS tasks:
//
//   * core 0 / Arduino loop -> Display refreshes (page updates, OTA progress)
//   * core 1 / IMU task     -> 200 Hz sensor reads
//
// Wire is not safe against that interleaving: a display transfer in the middle
// of a sensor register read makes the sensor return shifted or garbled bytes.
// Every bus access therefore goes through this mutex.
//
// The mutex is recursive because the drivers nest calls (Display::print(String)
// forwards to print(const char*), and AHRS reads the sensor while holding the
// lock to keep one sample atomic).

#include <Arduino.h>
#include <Wire.h>

namespace i2c
{

// Creates the mutex and starts the peripheral at I2C_CLOCK_HZ. Idempotent, so
// both Display::begin() and AHRS::begin() can call it.
bool begin();

// Acquires the bus. Returns false when the lock could not be taken within
// `timeoutMs`; the caller must then skip its transfer rather than block.
bool lock(uint32_t timeoutMs);

void unlock();

// RAII helper so an early return cannot leak the lock.
class Guard
{
public:
    explicit Guard(uint32_t timeoutMs);
    ~Guard();

    Guard(const Guard &) = delete;
    Guard &operator=(const Guard &) = delete;

    // False when the lock timed out; do not touch the bus in that case.
    bool locked() const { return acquired; }

private:
    bool acquired;
};

} // namespace i2c

#endif // I2C_BUS_H
